//
// Created by yuesang on 22-10-8.
//

#ifndef AUTOCHANGE_MIANWINDOW_H
#define AUTOCHANGE_MIANWINDOW_H

#include <QWidget>
#include <iostream>

QT_BEGIN_NAMESPACE
namespace Ui { class mianwindow; }
QT_END_NAMESPACE

class mianwindow : public QWidget {
Q_OBJECT

public:
    explicit mianwindow(QWidget *parent = nullptr);

    ~mianwindow() override;

private:
    Ui::mianwindow *ui;
    std::vector<std::vector<std::string>> rescult;
    void setcheck(std::vector<int> data);
    std::map<int,int> readcheck();

private slots:
    void openButton_Click();
    void ChangleButton_Click();
    void writepushButton_Click();


};


#endif //AUTOCHANGE_MIANWINDOW_H
