clear all
clc
close all

disp("Initialising ROS");

ipaddress = "192.168.0.100";
rosshutdown;
rosinit(ipaddress);

rgbImgSub = rossubscriber('/raspicam_node/image/compressed');   

compressedRGBImg = receive(rgbImgSub,5);

compressedRGBImg.Format = 'bgr8; jpeg compressed bgr8';

rgbImg = readImage(compressedRGBImg);

imwrite(rgbImg,'Img20.jpg')