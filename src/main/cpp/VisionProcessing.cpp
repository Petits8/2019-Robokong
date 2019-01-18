#include <VisionProcessing.h>


//Vision Processing will be done on the Raspberry PI 2
void VisionThread(){
     cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
        cv::Mat source;
        cv::Mat output;
        frc::Wait(5);
       while(true) {
          // frc::DriverStation::ReportError("Yeet");
           
            try{
                 cvSink.GrabFrame(source);
                cvtColor(source, output, cv::COLOR_BGR2GRAY);
                outputStreamStd.PutFrame(output);
            } catch(const char* msg){
                frc::DriverStation::ReportError(msg);
            }
            
            frc::Wait(.1);
        } 
    
}