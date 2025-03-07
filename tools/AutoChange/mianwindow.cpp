//
// Created by yuesang on 22-10-8.
//

// You may need to build the project (run Qt uic code generator) to get "ui_mianwindow.h" resolved

#include <QFileDialog>
#include "mianwindow.h"
#include "ui_mianwindow.h"
#include"Tool.h"
#include"map"

mianwindow::mianwindow(QWidget *parent) :
        QWidget(parent), ui(new Ui::mianwindow) {
    ui->setupUi(this);

    connect(ui->openButton, SIGNAL(clicked()),this,SLOT(openButton_Click()));
    connect(ui->changleButton, SIGNAL(clicked()),this,SLOT(ChangleButton_Click()));
    connect(ui->writepushButton, SIGNAL(clicked()),this,SLOT(writepushButton_Click()));
}

mianwindow::~mianwindow() {
    delete ui;
}

void mianwindow::setcheck(std::vector<int> data) {
    if(data[0]!=1){
        ui->BlueShaobing->setEnabled(false);
    }
    if(data[1]!=1){
        ui->BlueOne->setEnabled(false);
    }
    if(data[2]!=1){
        ui->BlueTwo->setEnabled(false);
    }
    if(data[3]!=1){
        ui->BlueThree->setEnabled(false);
    }
    if(data[4]!=1){
        ui->BlueFour->setEnabled(false);
    }
    if(data[5]!=1){
        ui->BlueFive->setEnabled(false);
    }
    if(data[6]!=1){
        ui->BlueQianshaozhan->setEnabled(false);
    }
    if(data[7]!=1){
        ui->BlueJidi->setEnabled(false);
    }
    if(data[8]!=1){
        ui->BlueJidiArmor->setEnabled(false);
    }


    if(data[9]!=1){
        ui->RedShaobing->setEnabled(false);
    }
    if(data[10]!=1){
        ui->RedOne->setEnabled(false);
    }
    if(data[11]!=1){
        ui->RedTwo->setEnabled(false);
    }
    if(data[12]!=1){
        ui->RedThree->setEnabled(false);
    }
    if(data[13]!=1){
        ui->RedFour->setEnabled(false);
    }
    if(data[14]!=1){
        ui->RedFive->setEnabled(false);
    }
    if(data[15]!=1){
        ui->RedQianshaozhan->setEnabled(false);
    }
    if(data[16]!=1){
        ui->RedJidi->setEnabled(false);
    }
    if(data[17]!=1){
        ui->RedJidiArmor->setEnabled(false);
    }



    if(data[18]!=1){
        ui->GrayShaobing->setEnabled(false);
    }
    if(data[19]!=1){
        ui->GrayOne->setEnabled(false);
    }
    if(data[20]!=1){
        ui->GrayTwo->setEnabled(false);
    }
    if(data[21]!=1){
        ui->GrayThree->setEnabled(false);
    }
    if(data[22]!=1){
        ui->GrayFour->setEnabled(false);
    }
    if(data[23]!=1){
        ui->GrayFive->setEnabled(false);
    }
    if(data[24]!=1){
        ui->GrayQianshaozhan->setEnabled(false);
    }
    if(data[25]!=1){
        ui->GrayJidi->setEnabled(false);
    }
    if(data[26]!=1){
        ui->GrayJidiArmor->setEnabled(false);
    }


    if(data[27]!=1){
        ui->PurpleShaobing->setEnabled(false);
    }
    if(data[28]!=1){
        ui->PurpleOne->setEnabled(false);
    }
    if(data[29]!=1){
        ui->PurpleTwo->setEnabled(false);
    }
    if(data[30]!=1){
        ui->PurpleThree->setEnabled(false);
    }
    if(data[31]!=1){
        ui->PurpleFour->setEnabled(false);
    }
    if(data[32]!=1){
        ui->PurpleFive->setEnabled(false);
    }
    if(data[33]!=1){
        ui->PurpleQianshaozhan->setEnabled(false);
    }
    if(data[34]!=1){
        ui->PurpleJidi->setEnabled(false);
    }
    if(data[35]!=1){
        ui->PurpleJidiArmor->setEnabled(false);
    }
}

void mianwindow::openButton_Click() {
    QString fileName = QFileDialog::getExistingDirectory(this, tr("选择数据集路径"), "/media/yuesang/G/Robotmaster/shangjiao/temp/");
    ui->readFileEdit->setText(fileName);
    auto temp=readfile(fileName.toStdString(),"txt");
    std::vector<std::vector<std::string>> data;
    for(int i=0;i<temp.size();i++){
        std::ifstream infile;
        infile.open(fileName.toStdString()+"/"+temp[i],std::ios::in);
        std::vector<std::string> temp0;
        std::string temp1;
        temp0.push_back(temp[i]);
        while (getline(infile,temp1)){
            temp0.push_back(temp1);
        }
        data.emplace_back(temp0);
    }
    auto tempclasses=getClasses(data);
    setcheck(tempclasses);
    this->rescult=data;

}

std::map<int,int> mianwindow::readcheck() {
    std::map<int,int> rescult;
    int count=0;
    if(ui->BlueShaobing->isChecked()){
        rescult.insert(std::pair<int, int>(0, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(0, -1));
    }
    if(ui->BlueOne->isChecked()){
        rescult.insert(std::pair<int, int>(1, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(1, -1));
    }
    if(ui->BlueTwo->isChecked()){
        rescult.insert(std::pair<int, int>(2, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(2, -1));
    }
    if(ui->BlueThree->isChecked()){
        rescult.insert(std::pair<int, int>(3, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(3, -1));
    }
    if(ui->BlueFour->isChecked()){
        rescult.insert(std::pair<int, int>(4, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(4, -1));
    }
    if(ui->BlueFive->isChecked()){
        rescult.insert(std::pair<int, int>(5, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(5, -1));
    }
    if(ui->BlueQianshaozhan->isChecked()){
        rescult.insert(std::pair<int, int>(6, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(6, -1));
    }
    if(ui->BlueJidi->isChecked()){
        rescult.insert(std::pair<int, int>(7, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(7, -1));
    }
    if(ui->BlueJidiArmor->isChecked()){
        rescult.insert(std::pair<int, int>(8, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(8, -1));
    }

    if(ui->RedShaobing->isChecked()){
        rescult.insert(std::pair<int, int>(9, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(9, -1));
    }
    if(ui->RedOne->isChecked()){
        rescult.insert(std::pair<int, int>(10, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(10, -1));
    }
    if(ui->RedTwo->isChecked()){
        rescult.insert(std::pair<int, int>(11, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(11, -1));
    }
    if(ui->RedThree->isChecked()){
        rescult.insert(std::pair<int, int>(12, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(12, -1));
    }
    if(ui->RedFour->isChecked()){
        rescult.insert(std::pair<int, int>(13, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(13, -1));
    }
    if(ui->RedFive->isChecked()){
        rescult.insert(std::pair<int, int>(14, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(14, -1));
    }
    if(ui->RedQianshaozhan->isChecked()){
        rescult.insert(std::pair<int, int>(15, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(15, -1));
    }
    if(ui->RedJidi->isChecked()){
        rescult.insert(std::pair<int, int>(16, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(16, -1));
    }
    if(ui->RedJidiArmor->isChecked()){
        rescult.insert(std::pair<int, int>(17, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(17, -1));
    }

    if(ui->GrayShaobing->isChecked()){
        rescult.insert(std::pair<int, int>(18, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(18, -1));
    }
    if(ui->GrayOne->isChecked()){
        rescult.insert(std::pair<int, int>(19, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(19, -1));
    }
    if(ui->GrayTwo->isChecked()){
        rescult.insert(std::pair<int, int>(20, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(20, -1));
    }
    if(ui->GrayThree->isChecked()){
        rescult.insert(std::pair<int, int>(21, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(21, -1));
    }
    if(ui->GrayFour->isChecked()){
        rescult.insert(std::pair<int, int>(22, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(22, -1));
    }
    if(ui->GrayFive->isChecked()){
        rescult.insert(std::pair<int, int>(23, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(23, -1));
    }
    if(ui->GrayQianshaozhan->isChecked()){
        rescult.insert(std::pair<int, int>(24, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(24, -1));
    }
    if(ui->GrayJidi->isChecked()){
        rescult.insert(std::pair<int, int>(25, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(25, -1));
    }
    if(ui->GrayJidiArmor->isChecked()){
        rescult.insert(std::pair<int, int>(26, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(26, -1));
    }

    if(ui->PurpleShaobing->isChecked()){
        rescult.insert(std::pair<int, int>(27, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(27, -1));
    }
    if(ui->PurpleOne->isChecked()){
        rescult.insert(std::pair<int, int>(28, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(28, -1));
    }
    if(ui->PurpleTwo->isChecked()){
        rescult.insert(std::pair<int, int>(29, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(29, -1));
    }
    if(ui->PurpleThree->isChecked()){
        rescult.insert(std::pair<int, int>(30, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(30, -1));
    }
    if(ui->PurpleFour->isChecked()){
        rescult.insert(std::pair<int, int>(31, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(31, -1));
    }
    if(ui->PurpleFive->isChecked()){
        rescult.insert(std::pair<int, int>(32, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(32, -1));
    }
    if(ui->PurpleQianshaozhan->isChecked()){
        rescult.insert(std::pair<int, int>(33, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(33, -1));
    }
    if(ui->PurpleJidi->isChecked()){
        rescult.insert(std::pair<int, int>(34, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(34, -1));
    }
    if(ui->PurpleJidiArmor->isChecked()){
        rescult.insert(std::pair<int, int>(35, count));
        count++;
    }else{
        rescult.insert(std::pair<int, int>(35, -1));
    }
    return rescult;
}

void mianwindow::ChangleButton_Click() {
    auto temp=readcheck();
    std::cout<<temp.size()<<std::endl;
//    for(auto kv:temp){
//        std::cout<<kv.first<<" "<<kv.second<<std::endl;
//    }
    for(int i=0;i<rescult.size();i++){
        std::ofstream onfile((ui->WriteFileEdit->toPlainText()));
        for(int j=1;j<rescult[i].size();j++){
            std::cout<<rescult[i][j]<<std::endl;
        }
        std::cout<<std::endl;
    }
}

void mianwindow::writepushButton_Click() {
    QString fileName = QFileDialog::getExistingDirectory(this, tr("保存数据集路径"), "/media/yuesang/G/Robotmaster/shangjiao/my/");
    ui->WriteFileEdit->setText(fileName);
}
