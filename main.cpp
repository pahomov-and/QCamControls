#include <QApplication>
#include <QtDebug>

#include <signal.h>
#include <string.h>

#include "main_window.h"
#include "message_handler.h"

#include <libcamera/camera_manager.h>

void signalHandler([[maybe_unused]] int signal)
{
    qInfo() << "Exiting";
    qApp->quit();
}

OptionsParser::Options parseOptions(int argc, char *argv[]) {
    StreamKeyValueParser streamKeyValue;

    OptionsParser parser;
    parser.addOption(OptCamera, OptionString,
                     "Specify which camera to operate on", "camera",
                     ArgumentRequired, "camera");
    parser.addOption(OptHelp, OptionNone, "Display this help message",
                     "help");
    parser.addOption(OptRenderer, OptionString,
                     "Choose the renderer type {qt,gles} (default: qt)",
                     "renderer", ArgumentRequired, "renderer");
    parser.addOption(OptStream, &streamKeyValue,
                     "Set configuration of a camera stream", "stream", true);
    parser.addOption(OptVerbose, OptionNone,
                     "Print verbose log messages", "verbose");

    OptionsParser::Options options = parser.parse(argc, argv);
    if (options.isSet(OptHelp))
        parser.usage();

    return options;
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    int ret;
    OptionsParser::Options options = parseOptions(argc, argv);
    if (!options.valid())
        return EXIT_FAILURE;
    if (options.isSet(OptHelp))
        return 0;

    MessageHandler msgHandler(options.isSet(OptVerbose));

    struct sigaction sa = {};
    sa.sa_handler = &signalHandler;
    sigaction(SIGINT, &sa, nullptr);

    libcamera::CameraManager *cm = new libcamera::CameraManager();

    ret = cm->start();
    if (ret) {
        qInfo() << "Failed to start camera manager:"
                << strerror(-ret);
        return EXIT_FAILURE;
    }

    MainWindow *mainWindow = new MainWindow(cm, options);

    mainWindow->show();
    ret = a.exec();
    delete mainWindow;

    cm->stop();
    delete cm;

    return ret;
}
