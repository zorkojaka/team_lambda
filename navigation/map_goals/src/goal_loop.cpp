#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <time.h>

#include <cstring>

using namespace std;

ros::Publisher goal_pub;
ros::Publisher marker_pub;
ros::Subscriber map_sub;
ros::Subscriber red_sub;
ros::Subscriber grn_sub;
ros::Subscriber blu_sub;

float map_resolution = 0;
tf::Transform map_transform;
visualization_msgs::Marker redMarker;
bool redFound=false;


//Mat cv_map;
int size_x;
int size_y;

int poznamoring;
int poznamocilinder;
int zaporednimoski=1;
int inforingspol=0;
int infocilinderspol=0;


struct person{
int id;
int spol;
int starost;
int visina;

int inforing; //1 ima -1 nima 0 nedefiniran
int infocilinder;
int truth;  //1 resnica -1 laž 0 nedefiniran
int prisotnost; // 0 nedefiniran 1 prisoten -1 neprisoten

person *next;

};

//heads of 2lists for woman and man
person *manhead;
person *womanhead;



void addperson(int id, int spol, int starost, int visina){	
	person *kaz;
	person *nov;
	
	if(spol==1){//moski
		kaz=manhead;//zacetk moskega lista
		while(kaz->next != NULL){	
			kaz=kaz->next;
		}
		//dobimo da kaz kaže na zadnje el.
		
		kaz->next=new person;
		kaz=kaz->next;

		//VNOS PODATKOV
		kaz->id=id;	
		kaz->spol=1;
		kaz->starost=starost;
		kaz->visina=visina;
		kaz->inforing=0;
		kaz->infocilinder=0;
		kaz->truth=1;
		kaz->prisotnost=0;
		kaz->next=NULL;
	
			
	}else{//zenska
		printf("notr\n");
		kaz=womanhead;
		while(kaz->next != NULL){
			kaz=kaz->next;
		}
		//dobimo da kaz kaže na zadnje el.
		
		kaz->next=new person;
		kaz=kaz->next;
		
		//VNOS PODATKOV
		kaz->id=id;	
		kaz->spol=2;
		kaz->starost=starost;
		kaz->visina=visina;
		kaz->inforing=0;
		kaz->infocilinder=0;
		kaz->truth=0;
		kaz->prisotnost=0;
		kaz->next=NULL;
	}
	
}

void printlist(person *head){
person *kaz=head;
while(kaz->next != NULL){
	kaz=kaz->next;
	printf("Oseba:%d, spol: %d, starost: %d, visina;%d, inforing: %d, infocilinder: %d, truth: %d, prisotnost: %d\n",kaz->id,kaz->spol,kaz->starost, kaz->visina, kaz->inforing, kaz->infocilinder, kaz->truth, kaz->prisotnost);
	fflush(stdout);
	
}

}

//incializacija vseh oseb v lista man in woman
void spoznavanjeoseb(){

	//incializacija glav listov
	manhead = new person;
	manhead->id=0;
	manhead->spol=1;
	manhead->next=NULL;
		
	womanhead = new person;
	womanhead->id=0;
	womanhead->spol=2;
	womanhead->next=NULL;
	
	//DODAMO VSE OSEBE (id,spol(1=man,2=woman),starost,visina,inforing,infocilinder,truth; (1=true,-1=false,0=undefined))
	
	//MAN DODAMO JIH NA LIST manhead->Harry
	//1. Harry	
	addperson(1,1,18,165);
	//2. Peter
	addperson(2,1,25,182);
	//3. Elvis
	addperson(3,1,34,181);
	//4. Forrest	
	addperson(4,1,42,183);
	//5. Tesla	
	addperson(5,1,45,188);
	//6. Albert	
	addperson(6,1,71,180);
	

// WOMAN  DODAMO JIH NA NOV LIST womanhead->Ilka...
	//7. Ilka	
	addperson(7,2,26,172);
	//8. Adele	
	addperson(8,2,29,175);
	//9. Scarlet	
	addperson(9,2,31,160);
	//10. Lindsay	
	addperson(10,2,32,178);
	//11. Tina
	addperson(11,2,33,171);
	//12. Ellen	
	addperson(12,2,59,170);
}


void pogovor(struct person per){
	
	struct person *kaz;	
	int odg=0;

	if(per.spol==2){
		//pogovor ženske
		//System("rosrun sound_play say.py 'Hi. Are you a woman'");
		kaz=womanhead;
		if(odg==1){
			//POMENI DA GOVORI RESNICO
			while(kaz->id != per.id){
				kaz=kaz->next;
			}
			kaz->truth=1;
			kaz->prisotnost=1;
		}else{//LAŽE
			while(kaz->id != per.id){
				kaz=kaz->next;
			}
			kaz->truth=-1;
			kaz->prisotnost=1;
		}
		
	}else if(per.spol==1){
		//POGOVOR Z MOŠKIM

		kaz=manhead->next;
		while(kaz->id!=per.id){
			kaz=kaz->next;
		}
		kaz->prisotnost=1;


		if(zaporednimoski==1){
			zaporednimoski++;
			//pogovor s prvim moškega
			//System("rosrun sound_play say.py 'Is the person who knows which ring is magical a man?'");
			
			//odg="yes";//yes
			if(odg==1){
				//woman cant have info about magic rings
				kaz=womanhead->next;
				inforingspol=1;
				while(kaz!=NULL){
					kaz->inforing=-1;
					kaz=kaz->next;
				}
			}else{
				//man cant have info about magic rings
				kaz=manhead->next;
				inforingspol=2;
				while(kaz!=NULL){
					kaz->inforing=-1;
					kaz=kaz->next;
				}
			}
			
			//DRUGO UPRAŠANJE
			//System("rosrun sound_play say.py 'Is the person who knows which location is right a man?'");
			
			if(odg==1){
				//woman cant have info about location
				kaz=womanhead->next;
				infocilinderspol=1;
				while(kaz!=NULL){
					kaz->infocilinder=-1;
					kaz=kaz->next;
				}
			}else{
				//man cant have info about magic rings
				kaz=manhead->next;
				infocilinderspol=2;
				while(kaz!=NULL){
					kaz->infocilinder=-1;
					kaz=kaz->next;
				}
			}
		}
		//naprej je za drugega in tretjega moškega
		else if(zaporednimoski==2){
			zaporednimoski++;
			//drugi moski ugotovimo kdo ve za ring
			if(inforingspol==1){
				kaz=manhead;
			}else{
				kaz=womanhead;
			}
			//kaz po zanki kaže na ung
			while(kaz->inforing!=0){
				kaz=kaz->next;
			}

			//1.VPRAŠANJE
			//System("rosrun sound_play say.py 'does USE NAME know where is ring ?'");
			if(odg==1){
				kaz->inforing=1; 
			}else{
				kaz->inforing=-1;		
			}
			while(kaz->inforing!=0){
				kaz=kaz->next;
			}
			
			//2. VPRAŠANJE			
			//System("rosrun sound_play say.py 'does USE NAME know where is ring ?'");
			
		}else if(zaporednimoski>2){
			zaporednimoski++; 
			//tretji moski ugotovimo kdo ve za cilinder
			
			
		}
	}
	
	
}







int main(int argc, char** argv) {
	
	ros::init(argc, argv, "goal_loop");
    ros::NodeHandle n;
	
	struct person *kaz;

	//manhead->Harry->Peter->Elvis->Forrest->Tesla->Albert
	//womanhead->Ilka->Adele->Scarlet->Lindsay->Tina->Ellen	
	spoznavanjeoseb();


	
	printlist(manhead);
	printlist(womanhead);
	

	kaz=manhead->next;
	pogovor(*kaz);
	pogovor(*manhead->next->next);
	pogovor(*manhead->next->next->next);

	pogovor(*womanhead->next);
	pogovor(*womanhead->next->next);
	pogovor(*womanhead->next->next->next);	


	printlist(manhead);
	printlist(womanhead);
	//printf("zacetek2\n");
	//incializacija oseb
	//spoznavanjeoseb();
	
	
	/*
    while(ros::ok()) {
	int x=0,y=100;
	int faceCounter=0;
		//
		do{
    		x=rand()%size_x;
    		y=rand()%size_y;
    	}while(getValAt(x,y)==0);
		tf::Point pt((float)x * 0.05, (float)y * 0.05, 0.0);
		tf::Point transformed = map_transform * pt;
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.orientation.w = 1;
		goal.target_pose.pose.position.x = transformed.x();
		goal.target_pose.pose.position.y = transformed.y();
		goal.target_pose.header.stamp = ros::Time::now();


  		ROS_INFO("Sending goal location ...");
   		ac.sendGoal(goal);

		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved!!!");
			
		if(redFound){
			//publish red marker and goal underneath
			printf("RED found\n");
			
			
			

			tf::Stamped<tf::Pose> transformed_red_marker;
			map2BaseLinkListner.transformPose("map", redMarker.pose, transformed_red_marker);

			
			marker_pub.publish(redMarker);
		}
		}
		*/


		ros::spinOnce();

	
	
    return 0;

}
