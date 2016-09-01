#include <image>
#include <arm>

Challenge3::Algorithm(){
  int target;
  Image::Targets Picture;
  
  //It is an array which save the image and location.
  Picture.classify_and_locate();
  arm::Arm myarm;
  
  while(?)
  //Decide the target ex:cat.
  {
  target=Challenge3::Generate_target(Picture);
  location=Picture(target);
  myarm.(location);
  }
}
