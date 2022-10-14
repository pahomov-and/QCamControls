/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main_window.cpp - qcam - Main application window
 */

#include "main_window.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <filesystem>
#include <sys/mman.h>

#include <QComboBox>
#include <QCoreApplication>
#include <QFileDialog>
#include <QImage>
#include <QImageWriter>
#include <QInputDialog>
#include <QMutexLocker>
#include <QStandardPaths>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QtDebug>

#include <libcamera/camera_manager.h>
#include <libcamera/version.h>
#include <zconf.h>

#include "dng_writer.h"
#include "viewfinder_qt.h"

using namespace libcamera;

#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
/*
 * Qt::fixed was introduced in v5.14, and ::fixed deprecated in v5.15. Allow
 * usage of Qt::fixed unconditionally.
 */
namespace Qt {
    constexpr auto fixed = ::fixed;
} /* namespace Qt */
#endif


class TakePhotoEvent : public QEvent {
public:
    TakePhotoEvent()
            : QEvent(type()) {
    }

    static Type type() {
        static int type = QEvent::registerEventType();
        return static_cast<Type>(type);
    }
};

/**
 * \brief Custom QEvent to signal capture completion
 */
class CaptureEvent : public QEvent {
public:
    CaptureEvent()
            : QEvent(type()) {
    }

    static Type type() {
        static int type = QEvent::registerEventType();
        return static_cast<Type>(type);
    }
};

/**
 * \brief Custom QEvent to signal hotplug or unplug
 */
class HotplugEvent : public QEvent {
public:
    enum PlugEvent {
        HotPlug,
        HotUnplug
    };

    HotplugEvent(std::shared_ptr<Camera> camera, PlugEvent event)
            : QEvent(type()), camera_(std::move(camera)), plugEvent_(event) {
    }

    static Type type() {
        static int type = QEvent::registerEventType();
        return static_cast<Type>(type);
    }

    PlugEvent hotplugEvent() const { return plugEvent_; }

    Camera *camera() const { return camera_.get(); }

private:
    std::shared_ptr<Camera> camera_;
    PlugEvent plugEvent_;
};

void MainWindow::ShowPropertis(int id) {
    std::cout << "\nPropertis:\n";
    for (auto prop: cm_->cameras()[id]->properties()) {
        std::cout
                << "\tid: 0x" << std::hex << prop.first << std::dec
                << "\t " << prop.second.toString() << "\n";
    }

}

void MainWindow::ShowControls(int id) {
    std::cout << "\nControls:\n";
    for (auto cont: cm_->cameras()[id]->controls()) {
        std::cout
                << "\tid: 0x" << std::hex << cont.first->id() << std::dec
                << "\tname: " << cont.first->name()
                << "\t " << cont.second.toString() << "\n";
    }
}


MainWindow::MainWindow(CameraManager *cm, const OptionsParser::Options &options)
        : saveRaw_(nullptr), options_(options), cm_(cm), allocator_(nullptr),
          isCapturing_(false), captureTake_(TAKE_NO), countPhoto_(0) {
    int ret;

    /*
     * Initialize the UI: Create the toolbar, set the window title and
     * create the viewfinder widget.
     */
    createToolbars();
//	createSetup();

    title_ = "LibCamera " + QString::fromStdString(CameraManager::version());
    setWindowTitle(title_);
    connect(&titleTimer_, SIGNAL(timeout()), this, SLOT(updateTitle()));

    /* Renderer type Qt or GLES, select Qt by default. */
    std::string renderType = "qt";
    if (options_.isSet(OptRenderer))
        renderType = options_[OptRenderer].toString();

    if (renderType == "qt") {
        ViewFinderQt *viewfinder = new ViewFinderQt(this);
        connect(viewfinder, &ViewFinderQt::renderComplete,
                this, &MainWindow::queueRequest);
        viewfinder_ = viewfinder;
        setCentralWidget(viewfinder);
//#ifndef QT_NO_OPENGL
//	} else if (renderType == "gles") {
//		ViewFinderGL *viewfinder = new ViewFinderGL(this);
//		connect(viewfinder, &ViewFinderGL::renderComplete,
//			this, &MainWindow::queueRequest);
//		viewfinder_ = viewfinder;
//		setCentralWidget(viewfinder);
//#endif
    } else {
        qWarning() << "Invalid render type"
                   << QString::fromStdString(renderType);
        quit();
        return;
    }

    adjustSize();

    /* Hotplug/unplug support */
    cm_->cameraAdded.connect(this, &MainWindow::addCamera);
    cm_->cameraRemoved.connect(this, &MainWindow::removeCamera);

    /* Open the camera and start capture. */
    ret = openCamera();
    if (ret < 0) {
        quit();
        return;
    }

    initSetup();

    startStopAction_->setChecked(true);
}

MainWindow::~MainWindow() {
    if (camera_) {
        stopCapture();
        camera_->release();
        camera_.reset();
    }
}

bool MainWindow::event(QEvent *e) {
    if (e->type() == CaptureEvent::type()) {
        processCapture();
//        processTakePhoto();
        return true;
    } else if (e->type() == HotplugEvent::type()) {
        processHotplug(static_cast<HotplugEvent *>(e));
        return true;
    } else if (e->type() == TakePhotoEvent::type()) {
//        processTakePhoto();
        return true;
    }

    return QMainWindow::event(e);
}

int MainWindow::createToolbars() {
    QAction *action;

    toolbar_ = addToolBar("Main");

    /* Disable right click context menu. */
    toolbar_->setContextMenuPolicy(Qt::PreventContextMenu);

    /* Quit action. */
    action = toolbar_->addAction(QIcon::fromTheme("application-exit",
                                                  QIcon(":x-circle.svg")),
                                 "Quit");
    action->setShortcut(Qt::CTRL | Qt::Key_Q);
    connect(action, &QAction::triggered, this, &MainWindow::quit);

    /* Camera selector. */
    cameraCombo_ = new QComboBox();
    connect(cameraCombo_, QOverload<int>::of(&QComboBox::activated),
            this, &MainWindow::switchCamera);

    for (const std::shared_ptr<Camera> &cam: cm_->cameras())
        cameraCombo_->addItem(QString::fromStdString(cam->id()));

    toolbar_->addWidget(cameraCombo_);

    toolbar_->addSeparator();

    /* Start/Stop action. */
    iconPlay_ = QIcon::fromTheme("media-playback-start",
                                 QIcon(":play-circle.svg"));
    iconStop_ = QIcon::fromTheme("media-playback-stop",
                                 QIcon(":stop-circle.svg"));

    action = toolbar_->addAction(iconPlay_, "Start Capture");
    action->setCheckable(true);
    action->setShortcut(Qt::Key_Space);
    connect(action, &QAction::toggled, this, &MainWindow::toggleCapture);
    startStopAction_ = action;

    /* Save As... action. */
    action = toolbar_->addAction(QIcon::fromTheme("document-save-as",
                                                  QIcon(":save.svg")),
                                 "Save As...");
    action->setShortcut(QKeySequence::SaveAs);
    connect(action, &QAction::triggered, this, &MainWindow::saveImageAs);

//#ifdef HAVE_DNG
    /* Save Raw action. */
    action = toolbar_->addAction(QIcon::fromTheme("camera-photo",
                                                  QIcon(":aperture.svg")),
                                 "Save Raw");
    action->setShortcut(QKeySequence::Copy);
//	action->setEnabled(false);
//	action->setDisabled(false);
    connect(action, &QAction::triggered, this, &MainWindow::captureRaw);
    saveRaw_ = action;
//#endif

    return 0;
}

void MainWindow::handlePhotoTakeRAW() {
    captureTake_ = TAKE_RAW;
    countPhoto_ = photoNumTake_->value();
}

void MainWindow::handlePhotoTakePNG() {
    captureTake_ = TAKE_PNG;
    countPhoto_ = photoNumTake_->value();
}


int MainWindow::initSetup() {
//    SensorConfig::InitConfigs(camera_);

    currentSensorConfig = SensorConfig::configSensor;

    const ControlInfoMap &info = camera_->controls();

    setupDockWidget_ = new QDockWidget(this);
    setupDockWidget_->setFixedWidth(320);

    tabWidget_ = new QTabWidget();
    sensorVBox_ = new QVBoxLayout();

    toolBoxAll_ = new QToolBox();

    /* -= Setup sensor =- */
    QWidget *widgetSetupSensor_ = new QWidget();
    QWidget *widgetPhoto_ = new QWidget();

    QVBoxLayout *vBoxLayoutSensor_ = new QVBoxLayout();


    for (auto cont: camera_->controls()) {
//        std::cout
//                << "\tid: 0x" << std::hex << cont.first->id() << std::dec
//                << "\tname: " << cont.first->name()
//                << "\t " << cont.second.toString() << "\n";

        QHBoxLayout *qhBoxLayout = nullptr;

        unsigned int id = cont.first->id();

        switch (cont.first->type()) {

//            case ControlType::ControlTypeBool:
            case ControlType::ControlTypeInteger32: {
                qhBoxLayout = Factory::factoryBoxSensorSetupItem<int32_t>(
                        info,
                        cont.first->id(),
                        [id](const double value) {
                            int32_t v = value;
//                            SensorConfig::configSensor.exposureTime = value;
                            SensorConfig::queueControls.push({id, v});
                            std::cout << __FILE__ << ":" << __LINE__ << "\tval: " << v << "\n";
                        });
                break;
            }

            case ControlType::ControlTypeFloat: {
                qhBoxLayout = Factory::factoryBoxSensorSetupItem<float>(
                        info,
                        cont.first->id(),
                        [id](const double value) {
                            float v = value;
//                            SensorConfig::configSensor.exposureTime = value;
                            SensorConfig::queueControls.push({id, v});
                            std::cout << __FILE__ << ":" << __LINE__ << "\tval: " << v << "\n";
                        });
                break;
            }

                /*
                case ControlType::ControlTypeBool: {
                    qhBoxLayout = Factory::factoryBoxSensorSetupItem<bool>(
                            info,
                            cont.first->id(),
                            [id](const double value) {
                                bool v = value;
                                SensorConfig::queueControls.push({id, v});
                                std::cout << __FILE__ << ":" << __LINE__ << "\tval: " << v << "\n";
                            });
                    break;
                }
                 */

        }

        if (qhBoxLayout) vBoxLayoutSensor_->addLayout(qhBoxLayout);
    }



    /* Add controls */
/*
    vBoxLayoutSensor_->addLayout(Factory::factoryBoxSensorSetupItem<int>(
            info,
            controls::EXPOSURE_TIME,
            [&](const double value) {
                std::cout << "EXPOSURE_TIME Value: " << value << "\n";
                SensorConfig::configSensor.exposureTime = value;
            }));


    vBoxLayoutSensor_->addLayout(
            factory.factoryBoxSensorSetupItem<double>(
                    info,
                    controls::ANALOGUE_GAIN,
                    [&](const double value) {
                        std::cout << "ANALOGUE_GAIN Value: " << value << "\n";
                        SensorConfig::configSensor.analogueGain = value;
//                        isConfigureISP = true;
                    }));

    vBoxLayoutSensor_->addLayout(
            factory.factoryBoxSensorSetupItem<double>(
                    info,
                    controls::DIGITAL_GAIN,
                    [&](const double value) {
                        std::cout << "DIGITAL_GAIN Value: " << value << "\n";
                        SensorConfig::configSensor.digitalGain = value;
//                        isConfigureISP = true;

                    }));


    vBoxLayoutSensor_->addLayout(
            factory.factoryBoxSensorSetupItem<double>(
                    info,
                    controls::SATURATION,
                    [&](const double value) {
                        std::cout << "SATURATION Value: " << value << "\n";
                        SensorConfig::configSensor.saturation = value;

                    }));

    vBoxLayoutSensor_->addLayout(
            factory.factoryBoxSensorSetupItem<double>(
                    info,
                    controls::SHARPNESS,
                    [&](const double value) {
                        std::cout << "SHARPNESS Value: " << value << "\n";
                        SensorConfig::configSensor.sharpness = value;

                    }));
*/

    vBoxLayoutSensor_->addItem(new QSpacerItem(0, 0, QSizePolicy::Fixed, QSizePolicy::Expanding));

    widgetSetupSensor_->setLayout(vBoxLayoutSensor_);
    toolBoxAll_->addItem(widgetSetupSensor_, "Setup sensor");


    /* -= Setup photo =- */
    QWidget *widgetSetupPhoto_ = new QWidget();
    QVBoxLayout *vBoxLayoutPhoto_ = new QVBoxLayout();
    widgetSetupPhoto_->setLayout(vBoxLayoutPhoto_);

//    vBoxLayoutPhoto_->addLayout(qhBoxLayout);
    photoPrefix_ = new QLineEdit();
    vBoxLayoutPhoto_->addWidget(new QLabel("Filename prefix"));
    vBoxLayoutPhoto_->addWidget(photoPrefix_);

    photoDirectory_ = new QLineEdit(QStandardPaths::writableLocation(QStandardPaths::PicturesLocation));
    vBoxLayoutPhoto_->addWidget(new QLabel("Directory for saving photos"));
    vBoxLayoutPhoto_->addWidget(photoDirectory_);

    photoNumTake_ = new QSpinBox();
    photoNumTake_->setMinimum(1);
    photoTakeRaw_ = new QPushButton("Take RAW");
    photoTakePng_ = new QPushButton("Take PNG");
    vBoxLayoutPhoto_->addWidget(new QLabel("Take following number of photos "));
    vBoxLayoutPhoto_->addWidget(photoNumTake_);
    vBoxLayoutPhoto_->addWidget(photoTakeRaw_);
    vBoxLayoutPhoto_->addWidget(photoTakePng_);
    connect(photoTakeRaw_, SIGNAL (released()), this, SLOT (handlePhotoTakeRAW())); //
    connect(photoTakePng_, SIGNAL (released()), this, SLOT (handlePhotoTakePNG())); //


    vBoxLayoutPhoto_->addItem(new QSpacerItem(0, 0, QSizePolicy::Fixed, QSizePolicy::Expanding));
    toolBoxAll_->addItem(widgetSetupPhoto_, "Setup photo");


    sensorVBox_->addWidget(toolBoxAll_);
    tabWidget_->setTabPosition(QTabWidget::West);
    tabWidget_->setLayout(sensorVBox_);
    setupDockWidget_->setWidget(tabWidget_);


    addDockWidget(Qt::RightDockWidgetArea, setupDockWidget_);


//    float maxAnalogGain = info.at(controls::ANALOGUE_GAIN).max().get<float>();
//    float minAnalogGain = info.at(controls::ANALOGUE_GAIN).min().get<float>();
//
//    int32_t maxExposure = info.at(controls::EXPOSURE_TIME).max().get<int32_t>();
//    int32_t minExposure = info.at(controls::EXPOSURE_TIME).min().get<int32_t>();
//
//    analogGainSlider->setMaximum(decimalSlider * maxAnalogGain);
//    analogGainSlider->setMinimum(decimalSlider * minAnalogGain);
//    analogGainSlider->setSingleStep(1);
//
//    QObject::connect(analogGainSlider, &QSlider::valueChanged, [&](const int value) {
//        analogGainValue = (float) value / decimalSlider;
//        std::cout << "analogGainValue: " << analogGainValue << "\n";
//        analogGainEdit->setText(QString("%1").arg(analogGainValue, 0, 'f'));
//        isConfigureISP = true;
//    });
//
//
//    exposureSlider->setMaximum(maxExposure);
//    exposureSlider->setMinimum(minExposure);
//    exposureSlider->setSingleStep(1);
//
//    QObject::connect(exposureSlider, &QSlider::valueChanged, [&](const int value) {
//        exposureValue = value;
//        std::cout << "exposureValue: " << exposureValue << "\n";
//        analogGainEdit->setText(QString("%1").arg(exposureValue));
//        isConfigureISP = true;
//    });

//    analogGainSlider->connect()

//    auto a = info.at(controls::ANALOGUE_GAIN);

//    ControlType a_t = info.at(controls::ANALOGUE_GAIN).min().type();

//    std::cout << "ANALOGUE_GAIN: [" << a.min().toString() << " - " << a.max().toString() <<"]\n";
//    std::cout << "ANALOGUE_GAIN: [" << minAnalogGain << " - " << maxAnalogGain << "]\n";

    return 0;
}

void MainWindow::quit() {
    QTimer::singleShot(0, QCoreApplication::instance(),
                       &QCoreApplication::quit);
}

void MainWindow::updateTitle() {
    /* Calculate the average frame rate over the last period. */
    unsigned int duration = frameRateInterval_.elapsed();
    unsigned int frames = framesCaptured_ - previousFrames_;
    double fps = frames * 1000.0 / duration;

    /* Restart counters. */
    frameRateInterval_.start();
    previousFrames_ = framesCaptured_;

    setWindowTitle(title_ + " : " + QString::number(fps, 'f', 2) + " fps");
}

/* -----------------------------------------------------------------------------
 * Camera Selection
 */

void MainWindow::switchCamera(int index) {
    /* Get and acquire the new camera. */
    const auto &cameras = cm_->cameras();
    if (static_cast<unsigned int>(index) >= cameras.size())
        return;

    const std::shared_ptr<Camera> &cam = cameras[index];

    if (cam->acquire()) {
        qInfo() << "Failed to acquire camera" << cam->id().c_str();
        return;
    }

    qInfo() << "Switching to camera" << cam->id().c_str();

    /*
     * Stop the capture session, release the current camera, replace it with
     * the new camera and start a new capture session.
     */
    startStopAction_->setChecked(false);

    camera_->release();
    camera_ = cam;

    startStopAction_->setChecked(true);
}

std::string MainWindow::chooseCamera() {
    QStringList cameras;
    bool result;

    /* If only one camera is available, use it automatically. */
    if (cm_->cameras().size() == 1)
        return cm_->cameras()[0]->id();

    /* Present a dialog box to pick a camera. */
    for (const std::shared_ptr<Camera> &cam: cm_->cameras())
        cameras.append(QString::fromStdString(cam->id()));

    QString id = QInputDialog::getItem(this, "Select Camera",
                                       "Camera:", cameras, 0,
                                       false, &result);
    if (!result)
        return std::string();

    return id.toStdString();
}

int MainWindow::openCamera() {
    std::string cameraName;

    ShowPropertis(0);
    ShowControls(0);

    /*
     * Use the camera specified on the command line, if any, or display the
     * camera selection dialog box otherwise.
     */
    if (options_.isSet(OptCamera))
        cameraName = static_cast<std::string>(options_[OptCamera]);
    else
        cameraName = chooseCamera();

    if (cameraName == "")
        return -EINVAL;

    /* Get and acquire the camera. */
    camera_ = cm_->get(cameraName);
    if (!camera_) {
        qInfo() << "Camera" << cameraName.c_str() << "not found";
        return -ENODEV;
    }

    if (camera_->acquire()) {
        qInfo() << "Failed to acquire camera";
        camera_.reset();
        return -EBUSY;
    }

    std::cout << "camera_ ANALOGUE_GAIN: [" << camera_->controls().at(controls::ANALOGUE_GAIN).min().get<float>()
              << " - " << camera_->controls().at(controls::ANALOGUE_GAIN).max().get<float>() << "]\n";

    /* Set the combo-box entry with the currently selected Camera. */
    cameraCombo_->setCurrentText(QString::fromStdString(cameraName));

    return 0;
}

/* -----------------------------------------------------------------------------
 * Capture Start & Stop
 */

void MainWindow::toggleCapture(bool start) {
    if (start) {
        startCapture();
        startStopAction_->setIcon(iconStop_);
        startStopAction_->setText("Stop Capture");
    } else {
        stopCapture();
        startStopAction_->setIcon(iconPlay_);
        startStopAction_->setText("Start Capture");
    }
}

/**
 * \brief Start capture with the current camera
 *
 * This function shall not be called directly, use toggleCapture() instead.
 */
int MainWindow::startCapture() {
    StreamRoles roles = StreamKeyValueParser::roles(options_[OptStream]);
    int ret;

    /* Verify roles are supported. */
    switch (roles.size()) {
        case 1:
            if (roles[0] != StreamRole::Viewfinder) {
                qWarning() << "Only viewfinder supported for single stream";
                std::cout << "Only viewfinder supported for single stream\n";
                return -EINVAL;
            }
            break;
        case 2:
            if (roles[0] != StreamRole::Viewfinder ||
                roles[1] != StreamRole::Raw) {
                qWarning() << "Only viewfinder + raw supported for dual streams";
                std::cout << "Only viewfinder + raw supported for dual streams\n";
                return -EINVAL;
            }
            break;
        default:
            if (roles.size() != 1) {
                qWarning() << "Unsupported stream configuration";
                std::cout << "Unsupported stream configuration\n";
                return -EINVAL;
            }
            break;
    }

    /* Configure the camera. */
    config_ = camera_->generateConfiguration(roles);
    if (!config_) {
        qWarning() << "Failed to generate configuration from roles";
        return -EINVAL;
    }

    StreamConfiguration &vfConfig = config_->at(0);

    /* Use a format supported by the viewfinder if available. */
    std::vector<PixelFormat> formats = vfConfig.formats().pixelformats();
    for (const PixelFormat &format: viewfinder_->nativeFormats()) {
        auto match = std::find_if(formats.begin(), formats.end(),
                                  [&](const PixelFormat &f) {
                                      return f == format;
                                  });
        if (match != formats.end()) {
            vfConfig.pixelFormat = format;
            break;
        }
    }

    /* Allow user to override configuration. */
    if (StreamKeyValueParser::updateConfiguration(config_.get(),
                                                  options_[OptStream])) {
        qWarning() << "Failed to update configuration";
        return -EINVAL;
    }

    CameraConfiguration::Status validation = config_->validate();
    if (validation == CameraConfiguration::Invalid) {
        qWarning() << "Failed to create valid camera configuration";
        return -EINVAL;
    }

    if (validation == CameraConfiguration::Adjusted)
        qInfo() << "Stream configuration adjusted to "
                << vfConfig.toString().c_str();

    ret = camera_->configure(config_.get());
    if (ret < 0) {
        qInfo() << "Failed to configure camera";
        return ret;
    }

    /* Store stream allocation. */
    vfStream_ = config_->at(0).stream();
    if (config_->size() == 2)
        rawStream_ = config_->at(1).stream();
    else
        rawStream_ = nullptr;

    /* Configure the viewfinder. */
    ret = viewfinder_->setFormat(vfConfig.pixelFormat,
                                 QSize(vfConfig.size.width, vfConfig.size.height));
    if (ret < 0) {
        qInfo() << "Failed to set viewfinder format";
        return ret;
    }

    adjustSize();

    /* Configure the raw capture button. */
    if (saveRaw_)
        saveRaw_->setEnabled(config_->size() == 2);

    /* Allocate and map buffers. */
    allocator_ = new FrameBufferAllocator(camera_);
    for (StreamConfiguration &config: *config_) {
        Stream *stream = config.stream();

        ret = allocator_->allocate(stream);
        if (ret < 0) {
            qWarning() << "Failed to allocate capture buffers";
            goto error;
        }

        for (const std::unique_ptr<FrameBuffer> &buffer: allocator_->buffers(stream)) {
            /* Map memory buffers and cache the mappings. */
            const FrameBuffer::Plane &plane = buffer->planes().front();
            void *memory = mmap(NULL, plane.length, PROT_READ, MAP_SHARED,
                                plane.fd.get(), 0);
            mappedBuffers_[buffer.get()] = {memory, plane.length};

            /* Store buffers on the free list. */
            freeBuffers_[stream].enqueue(buffer.get());
        }
    }

    /* Create requests and fill them with buffers from the viewfinder. */
    while (!freeBuffers_[vfStream_].isEmpty()) {
        FrameBuffer *buffer = freeBuffers_[vfStream_].dequeue();

        std::unique_ptr<Request> request = camera_->createRequest();
        if (!request) {
            qWarning() << "Can't create request";
            ret = -ENOMEM;
            goto error;
        }

        ret = request->addBuffer(vfStream_, buffer);
        if (ret < 0) {
            qWarning() << "Can't set buffer for request";
            goto error;
        }

        requests_.push_back(std::move(request));
    }

    /* Start the title timer and the camera. */
    titleTimer_.start(2000);
    frameRateInterval_.start();
    previousFrames_ = 0;
    framesCaptured_ = 0;
    lastBufferTime_ = 0;

    ret = camera_->start();
    if (ret) {
        qInfo() << "Failed to start capture";
        goto error;
    }

    camera_->requestCompleted.connect(this, &MainWindow::requestComplete);

    /* Queue all requests. */
    for (std::unique_ptr<Request> &request: requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            qWarning() << "Can't queue request";
            goto error_disconnect;
        }
    }

    isCapturing_ = true;

    return 0;

    error_disconnect:
    camera_->requestCompleted.disconnect(this, &MainWindow::requestComplete);
    camera_->stop();

    error:
    requests_.clear();

    for (auto &iter: mappedBuffers_) {
        const MappedBuffer &buffer = iter.second;
        munmap(buffer.memory, buffer.size);
    }
    mappedBuffers_.clear();

    freeBuffers_.clear();

    delete allocator_;
    allocator_ = nullptr;

    return ret;
}

/**
 * \brief Stop ongoing capture
 *
 * This function may be called directly when tearing down the MainWindow. Use
 * toggleCapture() instead in all other cases.
 */
void MainWindow::stopCapture() {
    if (!isCapturing_)
        return;

    viewfinder_->stop();
    if (saveRaw_)
        saveRaw_->setEnabled(false);
    captureTake_ = TAKE_NO;

    int ret = camera_->stop();
    if (ret)
        qInfo() << "Failed to stop capture";

    camera_->requestCompleted.disconnect(this, &MainWindow::requestComplete);

    for (auto &iter: mappedBuffers_) {
        const MappedBuffer &buffer = iter.second;
        munmap(buffer.memory, buffer.size);
    }
    mappedBuffers_.clear();

    requests_.clear();
    freeQueue_.clear();

    delete allocator_;

    isCapturing_ = false;

    config_.reset();

    /*
     * A CaptureEvent may have been posted before we stopped the camera,
     * but not processed yet. Clear the queue of done buffers to avoid
     * racing with the event handler.
     */
    freeBuffers_.clear();
    doneQueue_.clear();

    titleTimer_.stop();
    setWindowTitle(title_);
}

/* -----------------------------------------------------------------------------
 * Camera hotplugging support
 */

void MainWindow::processHotplug(HotplugEvent *e) {
    Camera *camera = e->camera();
    HotplugEvent::PlugEvent event = e->hotplugEvent();

    if (event == HotplugEvent::HotPlug) {
        cameraCombo_->addItem(QString::fromStdString(camera->id()));
    } else if (event == HotplugEvent::HotUnplug) {
        /* Check if the currently-streaming camera is removed. */
        if (camera == camera_.get()) {
            toggleCapture(false);
            camera_->release();
            camera_.reset();
            cameraCombo_->setCurrentIndex(0);
        }

        int camIndex = cameraCombo_->findText(QString::fromStdString(camera->id()));
        cameraCombo_->removeItem(camIndex);
    }
}

void MainWindow::addCamera(std::shared_ptr<Camera> camera) {
    qInfo() << "Adding new camera:" << camera->id().c_str();
    QCoreApplication::postEvent(this,
                                new HotplugEvent(std::move(camera),
                                                 HotplugEvent::HotPlug));
}

void MainWindow::removeCamera(std::shared_ptr<Camera> camera) {
    qInfo() << "Removing camera:" << camera->id().c_str();
    QCoreApplication::postEvent(this,
                                new HotplugEvent(std::move(camera),
                                                 HotplugEvent::HotUnplug));
}

/* -----------------------------------------------------------------------------
 * Image Save
 */

void MainWindow::saveImageAs() {
    QImage image = viewfinder_->getCurrentImage();
    QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);

    QString filename = QFileDialog::getSaveFileName(this, "Save Image", defaultPath,
                                                    "Image Files (*.png *.jpg *.jpeg)");
    if (filename.isEmpty())
        return;

    QImageWriter writer(filename);
    writer.setQuality(95);
    writer.write(image);
}

void MainWindow::captureRaw() {
    captureTake_ = TAKE_RAW;
}

void MainWindow::processRaw(FrameBuffer *buffer,
                            [[maybe_unused]] const ControlList &metadata) {
#ifdef HAVE_DNG
    QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    QString filename = QFileDialog::getSaveFileName(this, "Save DNG", defaultPath,
                                                    "DNG Files (*.dng)");

    if (!filename.isEmpty()) {
        const MappedBuffer &mapped = mappedBuffers_[buffer];
        DNGWriter::write(filename.toStdString().c_str(), camera_.get(),
                         rawStream_->configuration(), metadata, buffer,
                         mapped.memory);
    }
#endif

    {
        QMutexLocker locker(&mutex_);
        freeBuffers_[rawStream_].enqueue(buffer);
    }
}

void MainWindow::processRawTakePhoto(FrameBuffer *buffer,
                                     [[maybe_unused]] const ControlList &metadata) {

//#ifdef HAVE_DNG
    QString defaultPath = photoDirectory_->text();
//    QString filename = defaultPath + "/" + photoPrefix_->text() + QDateTime::currentMSecsSinceEpoch() + ".dng";
    QString filename =
            defaultPath + "/" + photoPrefix_->text() + "_" + QString::number(QDateTime::currentMSecsSinceEpoch()) +
            ".dng";

    std::cout << "filename: " << filename.toStdString() << "\n";

    if (!filename.isEmpty()) {
        const MappedBuffer &mapped = mappedBuffers_[buffer];
        DNGWriter::write(filename.toStdString().c_str(), camera_.get(),
                         rawStream_->configuration(), metadata, buffer,
                         mapped.memory);
    }
//#endif

    if (countPhoto_ > 0) {
        std::cout << "countPhoto_: " << countPhoto_ << "\n";
        countPhoto_--;
        captureTake_ = TAKE_RAW;
    }

    {
        QMutexLocker locker(&mutex_);
        freeBuffers_[rawStream_].enqueue(buffer);
    }
}

/* -----------------------------------------------------------------------------
 * Request Completion Handling
 */

void MainWindow::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled)
        return;

    /*
     * We're running in the libcamera thread context, expensive operations
     * are not allowed. Add the buffer to the done queue and post a
     * CaptureEvent for the application thread to handle.
     */
    {
        QMutexLocker locker(&mutex_);
        doneQueue_.enqueue(request);
    }

    QCoreApplication::postEvent(this, new CaptureEvent);
}

void MainWindow::processCapture() {
    /*
     * Retrieve the next buffer from the done queue. The queue may be empty
     * if stopCapture() has been called while a CaptureEvent was posted but
     * not processed yet. Return immediately in that case.
     */
    Request *request;
    {
        QMutexLocker locker(&mutex_);
        if (doneQueue_.isEmpty())
            return;

        request = doneQueue_.dequeue();
    }

    /* Process buffers. */
    if (request->buffers().count(vfStream_))
        processViewfinder(request->buffers().at(vfStream_));

    if (request->buffers().count(rawStream_)) {
//        processRaw(request->buffers().at(rawStream_), request->metadata());
        processRawTakePhoto(request->buffers().at(rawStream_), request->metadata());


    }

    request->reuse();
    QMutexLocker locker(&mutex_);
    freeQueue_.enqueue(request);
}

void MainWindow::processViewfinder(FrameBuffer *buffer) {
    framesCaptured_++;

    const FrameMetadata &metadata = buffer->metadata();

    double fps = metadata.timestamp - lastBufferTime_;
    fps = lastBufferTime_ && fps ? 1000000000.0 / fps : 0.0;
    lastBufferTime_ = metadata.timestamp;

    qDebug().noquote()
            << QString("seq: %1").arg(metadata.sequence, 6, 10, QLatin1Char('0'))
            << "bytesused:" << metadata.planes()[0].bytesused
            << "timestamp:" << metadata.timestamp
            << "fps:" << Qt::fixed << qSetRealNumberPrecision(2) << fps;


//    QString defaultPath = photoDirectory_->text();
//    QString filename =
//            defaultPath + "/" + photoPrefix_->text() + "_" + QString::number(QDateTime::currentMSecsSinceEpoch()) +
//            ".png";
//
//    std::cout << "filename: " << filename.toStdString() << "\n";
//
//
//    QImage image = viewfinder_->getCurrentImage();
//    QImageWriter writer(filename);
//    writer.setQuality(100);
//    writer.write(image);

    if (countPhoto_ > 0 && captureTake_ == TAKE_PNG) {
        std::cout << "countPhoto_: " << countPhoto_ << "\n";

        QString defaultPath = photoDirectory_->text();
        QString filename =
                defaultPath + "/" + photoPrefix_->text() + "_" + QString::number(QDateTime::currentMSecsSinceEpoch()) +
                ".png";

        std::cout << "filename: " << filename.toStdString() << "\n";

        std::filesystem::create_directory(defaultPath.toStdString());


        QImage image = viewfinder_->getCurrentImage();
        QImageWriter writer(filename);
        writer.setQuality(100);
        writer.write(image);

        countPhoto_--;
        captureTake_ = TAKE_PNG;
    }

    /* Render the frame on the viewfinder. */
    viewfinder_->render(buffer, &mappedBuffers_[buffer]);
}


void MainWindow::queueRequest(FrameBuffer *buffer) {
    Request *request;
    {
        QMutexLocker locker(&mutex_);
        if (freeQueue_.isEmpty())
            return;

        request = freeQueue_.dequeue();
    }

    request->addBuffer(vfStream_, buffer);


    if (captureTake_ == TAKE_RAW) {
        FrameBuffer *rawBuffer = nullptr;

        {
            QMutexLocker locker(&mutex_);
            if (!freeBuffers_[rawStream_].isEmpty())
                rawBuffer = freeBuffers_[rawStream_].dequeue();
        }

        if (rawBuffer) {
            request->addBuffer(rawStream_, rawBuffer);
//            captureTake_ = TAKE_NO;
        } else {
            qWarning() << "No free buffer available for RAW capture";
        }

        captureTake_ = TAKE_NO;
    }

//	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
//    request->controls().set(controls::ExposureTime, 300000);

/*
    if (currentSensorConfig != SensorConfig::configSensor) {

        if (currentSensorConfig.analogueGain != SensorConfig::configSensor.analogueGain)
            request->controls().set(controls::AnalogueGain, SensorConfig::configSensor.analogueGain);

        if (currentSensorConfig.digitalGain != SensorConfig::configSensor.digitalGain)
            request->controls().set(controls::DigitalGain, SensorConfig::configSensor.digitalGain);

        if (currentSensorConfig.exposureTime != SensorConfig::configSensor.exposureTime)
            request->controls().set(controls::ExposureTime, SensorConfig::configSensor.exposureTime);

        if (currentSensorConfig.saturation != SensorConfig::configSensor.saturation)
            request->controls().set(controls::Saturation, SensorConfig::configSensor.saturation);

        if (currentSensorConfig.sharpness != SensorConfig::configSensor.sharpness)
            request->controls().set(controls::Sharpness, SensorConfig::configSensor.sharpness);


    }
*/


//    std::cout << "queue len: " <<  SensorConfig::queueControls.size() << "\n";

    while (!SensorConfig::queueControls.empty()) {
        auto item = SensorConfig::queueControls.front();


//        std::function<void(ValueConf&)> visit = [](auto& arg) {
//            using T = std::decay_t<decltype(arg)>;
//
//            std::cout << typeid(decltype(arg)).name() << "\n";
//
//            if constexpr (std::is_same_v<T, std::string>) {
//                printf("string: %s\n", arg.c_str());
//                // ...
//            }
//            else if constexpr (std::is_same_v<T, int32_t>) {
//                printf("integer: %d\n", arg);
//                // ...
//            }
//            else if constexpr (std::is_same_v<T, bool>) {
//                printf("bool: %d\n", arg);
//                // ...
//            } else if constexpr (std::is_same_v<T, float>) {
//                printf("float: %f\n", arg);
//                // ...
//            } else {printf("NONE\n");}
//        };
//
//        visit(item.second);



        if (std::holds_alternative<int32_t>(item.second)) {
            int32_t v = std::get<int32_t>(item.second);
            request->controls().set(item.first, v);
            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":" << v << "\n";

        } else if (std::holds_alternative<float>(item.second)) {
            float v = std::get<float>(item.second);
            request->controls().set(item.first, v);
            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":" << v << "\n";

        }
        if (std::holds_alternative<bool>(item.second)) {
            bool v = std::get<bool>(item.second);
            request->controls().set(item.first, v);
            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":" << v << "\n";

        }


//
////        using T = std::decay_t<decltype(item.second)>;
//        using T = decltype(item.second);
//
//
//        if (std::is_same_v<T, int32_t>) {
//            int32_t v = std::get<int32_t>(item.second);
//            request->controls().set(item.first, v);
//            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":"<< v <<"\n";
//
//        } else if (std::is_same_v<T, float>) {
//            float v = std::get<float>(item.second);
//            request->controls().set(item.first, v);
//            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":"<< v <<"\n";
//
//        } if (std::is_same_v<T, bool>) {
//            bool v = std::get<bool>(item.second);
//            request->controls().set(item.first, v);
//            std::cout << "!!!!!!!!!!!!!!!" << item.first << ":"<< v <<"\n";
//
//        }

        SensorConfig::queueControls.pop();
    }


//    if (isConfigureISP) {
//        request->controls().set(controls::AnalogueGain, analogGainValue);
//        request->controls().set(controls::ExposureTime, exposureValue);
//        isConfigureISP = false;
//    }
//    request->controls().set(controls::AnalogueGain, 0);

//    std::cout << "ExposureTime: " <<  (float)request->controls().get(controls::ExposureValue) << "\n";

    camera_->queueRequest(request);
}
