
#include "cam/FlowVector.h"

int main(){
	FlowVector f;
	f.detected_obstacle();
	//cout<<"left:" + obstacle.first<<endl;
	//cout<<"right" + obstacle.second<<endl;
	String location = f.identify_location();
	cout<<location<<endl;
}








