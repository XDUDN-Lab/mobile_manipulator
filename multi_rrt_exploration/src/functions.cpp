#include "functions.h"


// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}


//Nearest function
std::vector<float> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x){

float min=Norm(V[0],x);
int min_index;
float temp;

for (int j=0;j<V.size();j++)
{
temp=Norm(V[j],x);
if (temp<=min){
min=temp;
min_index=j;}

}

return V[min_index];
}



//Steer function
std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta){
std::vector<float> x_new;

if (Norm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;
}





//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){

float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
int out;
out=Data[int(indx)];
return out;
}




// ObstacleFree function-------------------------------------

char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub){
float rez=float(mapsub.info.resolution)*.2;
float stepz=int(ceil(Norm(xnew,xnear))/rez); 
std::vector<float> xi=xnear;
char  obs=0; char unk=0;
 
geometry_msgs::Point p;
for (int c=0;c<stepz;c++){
  xi=Steer(xi,xnew,rez);
  		

   if (gridValue(mapsub,xi) ==100){     obs=1; }
   
   if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}
  }
char out=0;
 xnew=xi;
 if (unk==1){  out=-1;}
 	
 if (obs==1){  out=0;}
 		
 if (obs!=1 && unk!=1){   out=1;}

 
 
 
 return out;

 }

bool getCompleteFrontier(geometry_msgs::Point& p, geometry_msgs::PointStamped& exploration_goal, nav_msgs::OccupancyGrid& mapsub) {

  float Xstartx = mapsub.info.origin.position.x;
  float Xstarty = mapsub.info.origin.position.y;
  float resolution = mapsub.info.resolution;
  unsigned int width = mapsub.info.width;
  unsigned int height = mapsub.info.height;

  unsigned int indx= (unsigned int)(floor((p.y-Xstarty)/resolution)*width+ floor((p.x-Xstartx)/resolution));

  //std::vector<unsigned int> out = nhood8(indx, mapsub);

  // center of the frontier
  float centerx = 0;
  float centery = 0;
  unsigned int size = 1;


  std::vector<signed char> data = mapsub.data;

  if (data[indx] != -1) {
    ROS_WARN("The real map is not match with the gridmap!");
    return false;
  }

  std::queue<unsigned int> queue;
  std::vector<bool> visited(width*height, false);

  // push the initial cell
  queue.push(indx);
  visited[indx] = true;

  while (!queue.empty()) {
    unsigned int idx = queue.front();
    queue.pop();

    for (unsigned int nbr : nhood8(idx, mapsub)) {
      if (isNewFrontierCell(nbr, mapsub, visited)) {
        visited[nbr] = true;
        queue.push(nbr);

        std::vector<float> point = pointOfIndex(mapsub, nbr);
        centerx += point[0];
        centery += point[1];
        size++;

        //std::cout<<"-----------------------\n" << point[0] << "," << point[1]<< std::endl;

      }
    }


  }

  exploration_goal.point.x = centerx/size;
  exploration_goal.point.y = centery/size;

  return true;
}

std::vector<unsigned int> nhood8(unsigned int indx, nav_msgs::OccupancyGrid& mapData) {

  //returns grid value at "Xp" location
  //map data:  100 occupied      -1 unknown       0 free
  unsigned int width = mapData.info.width;
  unsigned int height = mapData.info.height;
  //float indx=(  floor((p.y-Xstarty)/resolution)*width)+( floor((p.x-Xstartx)/resolution) );
  std::vector<unsigned int> out;
  //std::vector<float> Xp;
  //Xp.push_back(p.x);

  if (indx > width * height - 1) {
    ROS_WARN("Evaluating nhood for offmap point");
    return out;
  }

  if (indx % width > 0) {
    out.push_back(indx-1);
  }

  if (indx % width < (width-1)) {
    out.push_back(indx+1);
  }

  if (indx >= width) {
    out.push_back(indx - width);
  }

  if (indx < width * (height-1)) {
    out.push_back(indx+width);
  }

  if (indx % width > 0 && indx >= width) {
    out.push_back(indx-1-width);
  }

  if (indx % width > 0 && indx < width*(height-1)){
    out.push_back(indx-1+width);
  }

  if (indx % width < (width -1) && indx>=width) {
    out.push_back(indx+1-width);
  }

  if (indx % width < (width-1) && indx < width*(height-1)) {
    out.push_back(indx+1+width);
  }

  return out;
}


bool isNewFrontierCell(unsigned indx, nav_msgs::OccupancyGrid& mapData, const std::vector<bool>& visited) {
  std::vector<signed char> data = mapData.data;

  //map data:  100 occupied      -1 unknown       0 free
  if (data[indx] != -1 || visited[indx]) {
    return false;
  }

  for (unsigned int nbr : nhood8(indx, mapData)) {
    if (data[nbr] == 0) {
      return true;
    }
  }

  return false;
}

std::vector<float> pointOfIndex(nav_msgs::OccupancyGrid& mapData, unsigned int indx){
  std::vector<float> point;
  unsigned int width = mapData.info.width;
  float Xstartx = mapData.info.origin.position.x;
  float Xstarty = mapData.info.origin.position.y;
  float resolution = mapData.info.resolution;

  float x = Xstartx + (indx % width)*resolution;
  float y = Xstarty + (indx / width)*resolution;

  point.push_back(x);
  point.push_back(y);

  return point;
}
