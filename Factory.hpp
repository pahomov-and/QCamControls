// Created by tymbys on 12.06.2021.
//

#ifndef ASTROCAMERA_FACTORY_H
#define ASTROCAMERA_FACTORY_H

#include <iostream>
#include <functional>
#include <mutex>
#include <atomic>

#include <QVBoxLayout>
#include <QDockWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>

#include <libcamera/controls.h>
#include <libcamera/control_ids.h>

#include "SensorConfig.h"

using namespace libcamera;

class Factory {
public:

    template<typename T>
    static QHBoxLayout *factoryBoxSensorSetupItem(const libcamera::ControlInfoMap &infoMap,
                                                  unsigned int controlsKey,
                                                  std::function<void(const double)> call) {

        if(infoMap.find(controlsKey) == infoMap.end()) return nullptr;

        QHBoxLayout *boxItem = new QHBoxLayout();
        QVBoxLayout *vboxItem = new QVBoxLayout();

        const ControlId *id = infoMap.find(controlsKey)->first;
        const ControlInfo &info = infoMap.find(controlsKey)->second;

//        QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
//        spLeft.setHorizontalStretch(1);
//        boxItem->setSizeConstraint(QLayout::SetNoConstraint);

        boxItem->addWidget(new QLabel(QString::fromStdString(id->name())));

        QDoubleSpinBox *spin = new QDoubleSpinBox();
//        spin->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Maximum);
//        spin->setBaseSize(100, 100);
        boxItem->addWidget(spin);

        QLineEdit *edit = new QLineEdit();
        edit->setEnabled(false);
        //boxItem->addWidget(edit);

//        spin->installEventFilter([](){
//            std::cout << __FILE__ << ":" << __LINE__ << "\tevent: MouseButtonRelease"<<  "\n";
//        });

//        connect(spin, &QPushButton::clicked, [=]{ some_label->setText(button->text()); });
//
//        spin->event([](QEvent *event){
//            if(event->type()== QEvent::MouseButtonRelease) {
//                std::cout << __FILE__ << ":" << __LINE__ << "\tevent: MouseButtonRelease"<<  "\n";
//            }
//
//        });

        //boxItem->addWidget(vboxItem->widget());

        if constexpr (std::is_same_v<T, float>) {
            std::cout << __FILE__ << ":" << __LINE__ << "\tcontrolsKey: " << controlsKey <<  "\n";

            double max = info.max().get<float>();
            double min = info.min().get<float>();
            double val = 0; //= info.def().get<float>();

            val = info.def().get<float>();

//            switch (controlsKey) {
//                case controls::ANALOGUE_GAIN:
//                    val = info.def().get<int32_t>();
//                    break;
//                case controls::SHARPNESS:
//                    val = info.def().get<float>();
//                    break;
//
//                default:
//                    val = 0;//info.def().get<int32_t>();
//                    break;
//
//            }



//            if ( controlsKey == controls::COLOUR_CORRECTION_MATRIX) {
//                val = min;
//            } else {
//                val = info.def().get<float>();
//            }

//            QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
//            QDoubleSpinBox *spin = new QDoubleSpinBox();
//            boxItem->addWidget(spin);
//
//            QLineEdit *edit = new QLineEdit();
//            edit->setEnabled(false);
//            boxItem->addWidget(edit);

            spin->setMaximum(max);
            spin->setMinimum(min);
            spin->setValue(val);
            spin->setSingleStep(0.1);

            edit->setText(QString::fromStdString("[" + std::to_string(min) + "..." + std::to_string(max)));

//            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double d){ /* ... */ });
//            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), call);
/*
            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [edit](double d){
                edit->setText(QString::number(d,'g'));
            } );
*/

        } else if constexpr (std::is_same_v<T, int32_t>) {
            std::cout << __FILE__ << ":" << __LINE__ << "\tcontrolsKey: " << controlsKey <<  "\n";
            double max = info.max().get<int32_t>();
            double min = info.min().get<int32_t>();
            double val = 0;//= info.def().get<int32_t>();
            double step = 1;

//            if ( controlsKey == controls::AE_ENABLE) {
//                val = info.def().get<bool>();
//            } else {
//                val = info.def().get<int32_t>();
//            }

            switch (controlsKey) {
                case controls::AE_ENABLE:
                    val = info.def().get<bool>();
                    break;
                case controls::EXPOSURE_TIME:
//                    val = 100000; //info.def().get<int32_t>();
                    val = info.def().get<int32_t>();
                    step = val / 100; //10000;
                    break;

                default:
                    val = info.def().get<int32_t>();
                    break;
            }


            std::cout << __FILE__ << ":" << __LINE__ << "\tcontrolsKey: " << controlsKey <<"\tval: " << val << "\n";

//            boxItem->addWidget(new QLabel(QString::fromStdString(id->name())));
//            QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
//            boxItem->addWidget(slider);
//
//            QLineEdit *edit = new QLineEdit();
//            edit->setEnabled(false);
//            boxItem->addWidget(edit);

            spin->setMaximum(max);
            spin->setMinimum(min);
            spin->setValue(val);
            spin->setSingleStep(step);

            edit->setText(QString::fromStdString("[" + std::to_string(min) + "..." + std::to_string(max)));

//            SensorConfig::configSensor.

//            QObject::connect(slider, &QSlider::valueChanged, call);
//            QObject::connect(slider, QOverload<T>::of(&QSlider::valueChanged), call);
//            QObject::connect(slider, QOverload<T>::of(&QSlider::valueChanged), [edit](int d){
//                edit->setText(QString::number(d));
//            } );

//            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), call);

            /*
            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [edit](double i){
                edit->setText(QString::number(i));
            } );
            */

        } else if constexpr (std::is_same_v<T, bool>) {
            std::cout << __FILE__ << ":" << __LINE__ << "\tcontrolsKey: " << controlsKey <<  "\n";
            bool max = info.max().get<bool>();
            bool min = info.min().get<bool>();
//            bool val = info.def().get<bool>();

//            boxItem->addWidget(new QLabel(QString::fromStdString(id->name())));
//            QSlider *slider = new QSlider(Qt::Orientation::Horizontal);
//            boxItem->addWidget(slider);
//
//            QLineEdit *edit = new QLineEdit();
//            edit->setEnabled(false);
//            boxItem->addWidget(edit);

            spin->setMaximum(max);
            spin->setMinimum(min);
            spin->setValue(min);
            spin->setSingleStep(1);

//            SensorConfig::configSensor.

//            QObject::connect(slider, &QSlider::valueChanged, call);
//            QObject::connect(slider, QOverload<T>::of(&QSlider::valueChanged), call);
//            QObject::connect(slider, QOverload<T>::of(&QSlider::valueChanged), [edit](int d){
//                edit->setText(QString::number(d));
//            } );

//            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), call);

            /*
            QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [edit](double i){
                edit->setText(QString::number(i));
            } );
             */

        }

        QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), call);

//        QObject::connect(spin, QOverload<T>::of(&QDoubleSpinBox::valueChanged), call);
//        QObject::connect(spin, QOverload<T>::of(&QDoubleSpinBox::valueChanged), [edit](double d){
//            edit->setText(QString::number(d,'g'));
//        } );

        return boxItem;
    }


};


#endif //ASTROCAMERA_FACTORY_H
