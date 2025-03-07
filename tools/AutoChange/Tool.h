//
// Created by yuesang on 22-10-8.
//
#pragma once
#ifndef AUTOCHANGE_TOOL_H
#define AUTOCHANGE_TOOL_H

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <fstream>

std::vector<std::string> readfile(const std::string filename,const std::string format){
    struct dirent *ptr;
    DIR *dir;
    std::string PATH = filename;
    dir=opendir(PATH.c_str());
    std::vector<std::string> files;
    while((ptr=readdir(dir))!=NULL)
    {
        //跳过'.'和'..'两个目录
        if(ptr->d_name[0] == '.')
            continue;
        //cout << ptr->d_name << endl;
        files.push_back(ptr->d_name);
    }
    std::vector<std::string> rescult;
    for (int i = 0; i < files.size(); ++i)
    {
       int n=files[i].find(".");
       std::string temp=files[i].substr(n+1,files[i].size());
       if(temp==format){
           rescult.emplace_back(files[i]);
       }
    }

    closedir(dir);
    return rescult;
}

std::vector<std::string> split(std::string str, std::string pattern=" ")
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;//扩展字符串以方便操作
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size)
        {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

std::vector<int> getClasses(std::vector<std::vector<std::string>> data){
    std::vector<int> classes(36,0);
    for(int i=0;i<data.size();i++){
        for(int j=1;j<data[i].size();j++){
            auto rescult=split(data[i][j]);
            auto classesid=atoi(rescult[0].c_str());;
            classes[classesid]=1;
        }
    }
    return classes;
}

#endif //AUTOCHANGE_TOOL_H
