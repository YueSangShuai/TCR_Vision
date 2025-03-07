#include <QApplication>
#include <QPushButton>
#include "mianwindow.h"
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    mianwindow ui;
    ui.show();

    return QApplication::exec();
}
