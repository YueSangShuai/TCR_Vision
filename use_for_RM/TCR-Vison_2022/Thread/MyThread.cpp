#include "MyThread.h"
#include "../Kalman/Kalman.h"

std::vector<double> x,xtrue,xkalman,x3,ytrue,ykalman,y3,ztrue,zkalman,z3;
int counts=0;
double p1 = 0;

void RePicture(const int &enemy_Color) {//读图片
    string name="./armor_Img/img.png";
    Mat src = imread(name);
    if (src.empty()) {
        printf("could not load image...\n");
    }
    imshow("dst",src);
    waitKey(0);

}//读图片

void ReVideo(const int &enemy_Color) {//读视频
    if(debug_level>=1)
        cout << "is begin" << endl;
    MyThread myThread;
    myThread.setColor(enemy_Color);
    thread *Re_thread = new thread(std::bind(&MyThread::VInthread, &myThread));//读取相机
    thread *Os_thread = new thread(std::bind(&MyThread::decThread, &myThread));//识别

    Re_thread->join();
    Os_thread->join();
    if(debug_level>=1)
        cout << "is over" << endl;
}

void ReCamera(const int &enemy_Color) {//相机
    if(debug_level>=1)
        cout << "is begin" << endl;
    Mat frame;//图像
    MyThread myThread;//定义线程颜色
    myThread.setColor(enemy_Color);



    thread *Re_thread = new thread(std::bind(&MyThread::CInthread, &myThread));
    thread *Os_thread = new thread(std::bind(&MyThread::decThread, &myThread));
    thread *RePort_thread = new thread(std::bind(&MyThread::ReceivePort, &myThread));
    thread *WePort_thread = new thread(std::bind(&MyThread::SendPort, &myThread));
    thread *PrePort_thread = new thread(std::bind(&MyThread::Prethread, &myThread));
    RePort_thread->join();
    WePort_thread->join();
    Re_thread->join();
    Os_thread->join();
    PrePort_thread->join();


    if(debug_level>=1)
        cout << "is over" << endl;
}//读相机

//读取串口线程
void MyThread::SendPort() {
    int pit=0,yaw=0;
    while(isContinue)//时刻接受数据
    {
        while(!analyzeData.isSend){
            ;
        }

        serialPort.send_data(analyzeData.TransData());
        analyzeData.isSend=false;

    }
    if(debug_level>=1)
        cout<<"程序结束"<<endl;
    exit(1);
}

//发送串口线程
void MyThread::ReceivePort() {
    while(isContinue)//时刻接收数据
    {
        analyzeData.setData(serialPort.read_data());//设置数据
        color = analyzeData.getColor();
        //设置参数
        writeFrame(analyzeData.getColor());
    }
}

//读取线程   读视频
void MyThread::VInthread() {
    VideoCapture videoCapture;
    string name = "./armor_Img/testVideo.avi";
    videoCapture.open(name);
    if (!videoCapture.isOpened()) {
        printf("could not read this video file...\n");
        exit(1);
    }
    int c=0;
    while(isContinue){
        unique_lock<mutex> lockGuard(mutexLock);//加锁
        while(!frame.empty()){//如果不为空
            //cout<<"当前相机有东西"<<endl;
            CarmeraCondition.wait(lockGuard);//相机等待
        }
        if(!videoCapture.read(frame))//获取图像
        {break;}
        setImg(frame);
        c=waitKey(1);
        if(c==27){
            isContinue=false;
        }
        lockGuard.unlock();//  解锁
        DetectCondition.notify_all();//告知检测条件不用等待了
    }
    videoCapture.release();//释放相机
    destroyAllWindows();//销毁所有窗口
    exit(0);
}


//读取线程   开相机
void MyThread::CInthread() {

    auto config=YAML::LoadFile("/home/yuesang/Project/CLionProjects/Toauto_aimHK/carmeraYAML/camera.yaml");

    // 设置分辨率
    HaiKang.SetResolution(config["ResolutionW"].as<int>(), config["ResolutionH"].as<int>());
    // 开始采集帧
    HaiKang.SetStreamOn();
    // 设置曝光事件
    HaiKang.SetExposureTime(config["ExposureTime"].as<int>());
    //增益设置
//    if(config["setGain"].as<bool>()){
//         HaiKang.SetGAIN(0, 16);
//         HaiKang.SetGAIN(1, 8);
//         HaiKang.SetGAIN(2, 8);
//         HaiKang.SetGAIN(3, 16);
//    }
    // 是否启用自动白平衡7
    if(config["setAutoGain"].as<bool>()){
        HaiKang.Set_Auto_BALANCE();
    }
    // manual白平衡 BGR->012
    if(config["Set_BALANCE"].as<bool>()){
        HaiKang.Set_BALANCE(0, config["B_Gain"].as<int>());
        HaiKang.Set_BALANCE(1, config["G_Gain"].as<int>());
        HaiKang.Set_BALANCE(2, config["R_Gain"].as<int>());
    }
    char c;
    while(isContinue)
    {
        unique_lock<mutex> lockGuard(mutexLock);//加锁
        while(!frame.empty()){//如果不为空
            CarmeraCondition.wait(lockGuard);//相机等待
        }
        HaiKang.GetMat(frame);
       // flip(frame,frame,-1);
//        flip(frame,frame,-1);
        if(SaveVideo_Ok)//存视频
            wri.write(frame);

        if(debug_level>=4)
            imshow("ewq",frame);

        c=waitKey(1);
        if(c==27)
            isContinue=false;
        lockGuard.unlock();//  解锁
        DetectCondition.notify_all();//告知检测条件不用等待了
    }
    setEmpty();
    destroyAllWindows();//销毁所有窗口
}


//识别线程
void MyThread::decThread() {
    double FPS = 0.1;
    TRTModule model("/home/yuesang/Project/CLionProjects/Toauto_aimHK/model/model-opt-3.onnx");//识别
    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    while(isContinue){
        unique_lock<mutex> lockGuard(mutexLock);//加锁
        if(SaveVideo_Ok)//存视频　不识别
            continue;
        while(frame.empty()){
            //cout<<"图像为空"<<endl;
            DetectCondition.wait(lockGuard);//检测等待
        }

        //获取识别到的装甲板
        std::vector<bbox_t> armors = model(frame);//结果
        cv::Mat im2show = frame.clone();
        if(!armors.empty()){//判断是否有装甲板
//            for (const auto &armor: armors) {
//                cv::line(im2show, armor.pts[0], armor.pts[2], colors[2], 2);
//                cv::line(im2show, armor.pts[1], armor.pts[3], colors[2], 2);
//                cv::putText(im2show, std::to_string(armor.tag_id), armor.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[armor.color_id]);
//            }
            //决策
            bbox_t armor;
            if(Decision(armors,armor)) //决策 返回true说明没有符合要求的 返回false说明找到符合要求的
            {
                if(armor.tag_id == 1) armor.type=1;
                else    armor.type = 0; //判断其是大装甲还是小装甲
                //绘制装甲板
                cv::line(im2show, armor.pts[0], armor.pts[2], colors[2], 2);
                cv::line(im2show, armor.pts[1], armor.pts[3], colors[2], 2);
                cv::putText(im2show, std::to_string(armor.tag_id), armor.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[armor.color_id]);

                Point3f point;
                if(pnpDistance.TosolvePnP(armor,point)){//pnp测距
//                    cv::putText(im2show, fmt::format("dis={}", round(point.z)), {10, 45}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
                    point.z /=5;
//                    point.x /=5;
//                    point.y /=5;
                    analyzeData.setPoint(point);//设置相机的平移点
                    //设置装甲板位置到世界坐标系中
                    analyzeData.analyze();//分析数据

//                    analyzeData.getrescult(0,0);
                    if(debug_level > 0){
                        pnpDistance.getrescult(analyzeData.SPit,analyzeData.SYaw);
////                        std::cout<<"pitc"<<analyzeData.SPit<<"yaw"<<analyzeData.SYaw<<std::endl;
////                       std::cout<<"WORLD"<<pnpDistance.armor2world.translation()()<<std::endl;
//                        Eigen::MatrixXd temp1=pnpDistance.armor2world.translation().transpose();
//                        Eigen::MatrixXd temp2=pnpDistance.armor2world.rotationMatrix();
//                        auto temp3=temp1*temp2;
//

                        int a=pnpDistance.world_Eigen(0);//x
                        int b=pnpDistance.world_Eigen(1);//y
                        int c=pnpDistance.world_Eigen(2);//z


                        counts++;




                        double tan_pitch = analyzeData.getPitch(c*0.001,-b*0.001);//弹道补偿
                        double tan_yaw = a*1.0 / c;
                        float  myYaw1= -analyzeData.getYaw()+analyzeData.SYaw;

                        float myPitch1 = -analyzeData.getPit() +analyzeData.SPit ;

                        float world_x = point.z * tan(myYaw1/57.2958f);
                        float world_y = point.z * tan(myPitch1/57.2958f);



                        double Ts = 1/FPS;
                        kalman.predict(Ts);
                        Eigen::MatrixXf Z_in=Eigen::MatrixXf(3,1);
                        Z_in<< world_x,world_y, point.z;

                        kalman.update(Z_in);

                        x.push_back(counts);

                        xtrue.push_back(world_x);
                        xkalman.push_back(kalman.get_x()(0));
                        x3.push_back(kalman.get_x()(1));

                        ytrue.push_back(world_y);
                        ykalman.push_back(kalman.get_x()(2));
                        y3.push_back(kalman.get_x()(3));

                        ztrue.push_back(point.z);
                        zkalman.push_back(kalman.get_x()(4));
                        z3.push_back(kalman.get_x()(5));

                        if(counts>=100){
                            x.erase(x.begin());
                            xtrue.erase(xtrue.begin());
                            xkalman.erase(xkalman.begin());
                            x3.erase(x3.begin());

                            ytrue.erase(ytrue.begin());
                            ykalman.erase(ykalman.begin());
                            y3.erase(y3.begin());

                            ztrue.erase(ztrue.begin());
                            zkalman.erase(zkalman.begin());
                            z3.erase(z3.begin());
                        }

//                        xtrue.push_back(myYaw1);
//                        xkalman.push_back(-analyzeData.getYaw());
//                        x3.push_back(world_x);
//                        x3.push_back(analyzeData.SYaw);

//                        ytrue.push_back(myPitch1);
//                        ykalman.push_back(-analyzeData.getPit());
//                        y3.push_back(world_y);
//                        y3.push_back(analyzeData.SPit);



//                        ztrue.push_back(point.z);
//                        zkalman.push_back(c);


//                        cv::putText(im2show, fmt::format("x={}", a), {10, 45}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("y={}", b), {10, 65}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("z={}", c), {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);

////
//                        cv::putText(im2show, fmt::format("x={}", int(point.x)), {120, 45}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("y={}", int(point.y)), {120, 65}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("z={}", int(point.z)), {120, 85}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("pit={}", myPitch1), {10, 145}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("yaw={}", myYaw1), {10, 165}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("pit={}", analyzeData.getPit()), {10, 45}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("yaw={}", analyzeData.getYaw()), {10, 65}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
//                        cv::putText(im2show, fmt::format("dis={}", int(point.z)), {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
                    }

                    analyzeData.isSend=true;
                }
            }
        }
        fps_count++;
        auto t2 = system_clock::now();
        if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
            fps = fps_count;
            fps_count = 0;
            t1 = t2;
        }
//        std::cout<<fps<<std::endl;
        if(debug_level > 0){
            cv::putText(im2show, fmt::format("clo={}", color), {10, 105}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
            cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
            cv::imshow("dst",im2show);
        }

//        cout<<"Detect"<<endl;
        setEmpty();//设置空
        CarmeraCondition.notify_all();//告知相机条件不用等待了
        lockGuard.unlock();//  解锁
    }
}


void MyThread::setImg(const Mat& img)  {
     //flip(img,img,-1);
     frame=img;
     if(debug_level>=3){
         imshow("input",img);
     }

}

void MyThread::setEmpty() {
    frame.release();//释放图片
}


void MyThread::setColor(const int &num) {
    color = num;
}

void MyThread::writeFrame(const int& type) {
    //存视频
    if(type==2){
        if(!SaveVideo_Ok){//判断第一次是否是False如果是　说明进行了修改则切换另一个视频
            time(&timeReal);
            timeReal = timeReal + 8*3600;
            tm* t = gmtime(&timeReal);
            string name = getTime(t);
            string filename = "/home/tcrnx/Project/Toauto_aimHK/cmake-build-debug/armor_Img/" + name + ".avi";
            wri.open(filename, encode_type, 60, Size(640,512),CV_8UC3);   //创建保存视频文件的视频流  img.size()为保存的视频图像的尺寸
        }
        SaveVideo_Ok = true;
    }else{
        SaveVideo_Ok = false;
    }
}

bool MyThread::Decision(vector <bbox_t> armors,bbox_t &armor) {
    std::vector<double> weights(armors.size());
    for(int i=0;i<armors.size();i++){
        if(armors[i].confidence<0.6 || color != armors[i].color_id || armors[i].color_id == 2){
            weights[i]=-1;
            continue;
        }

//        if(armors[i].tag_id == 2) weights[i]= 0.0;//工程的优先级
//        else if(armors[i].tag_id == 1) weights[i]=2.0;//英雄的优先级
//        else if(armors[i].tag_id >=3 && armors[i].tag_id<=5) weights[i]=1.0;//步兵优先级
//        else weights[i] = 3.0;//哨兵、基地、前哨战的优先级
//
//        //计算面积
//        double the_area = (armors[i].pts[3].x,armors[i].pts[1].x) *
//                   (armors[i].pts[3].y,armors[i].pts[1].y);
//        weights[i] += the_area/10000;
////        std::cout<<the_area<<std::endl;
//        if(armors[i].tag_id == last_id)
//            weights[i] +=5;
    }
    auto max_weight = std::max_element(weights.begin(),weights.end());//找到最大值
    if(*max_weight < 0){
        return false;//说明没有符合要求的装甲板
    }else{
        armor = armors[max_weight - weights.begin()];
        last_id = armor.tag_id;
        return true;//返回符合要求的装甲板的最大值
    }


}




string getTime(tm* t){//返回时间
    string res = to_string(t->tm_hour) + "_" + to_string(t->tm_min) + "_" + to_string(t->tm_sec) + "__" + to_string(t->tm_mon +1) + '_' + to_string(t->tm_mday);
    return res;
}

void MyThread::Prethread() {
    while(1){

//        plt::clf();
//        plt::cla();
//
//        plt::subplot2grid(3,1,0,0);
//        plt::named_plot("xtrue", x, xtrue);
//        plt::named_plot("xkal", x, xkalman);
//        plt::legend();
//        plt::title("x:");
//
//        plt::subplot2grid(3,1,1,0);
//        plt::named_plot("vx", x, x3);
////        plt::named_plot("y_camera", x, ykalman);
//
//        plt::legend();
//        plt::title("vx:");
//
////        plt::subplot2grid(3,1,1,0);
////        plt::named_plot("shijie", x, ytrue);
////        plt::named_plot("y_camera", x, ykalman);
////
////        plt::legend();
////        plt::title("y:");
////
////        plt::subplot2grid(3,1,2,0);
////        plt::named_plot("ztrue", x, ztrue);
////        plt::named_plot("zpredict", x, zkalman);
////        plt::legend();
////        plt::title("z:");
//
//
//
//////        // Add graph title
//
//        plt::pause(0.1);
    }
}
