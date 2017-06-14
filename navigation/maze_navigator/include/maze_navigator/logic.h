#ifndef LOGIC_H
#define LOGIC_H

#include <std_msgs/String.h>

void callbackodg(const std_msgs::StringConstPtr &odgovor);
void spoznavanjeoseb();
void pogovor(int id);
void printMenHead();
void printWomenHead();
int gotinfo();
void prisotnostoseb();

#endif

