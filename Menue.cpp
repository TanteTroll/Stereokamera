#include "StdAfx.h"
#include "Menue.h"

#include <pcl/io/io.h>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include <iostream> 
#include <sstream>  
#include <stdio.h>

Menue::Menue(void)
{
}


Menue::~Menue(void)
{
}

void Menue::printMenueEntry(int userEntry, const char* description, int value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %40s",description);
	pcl::console::print_value(" %5d ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntry(int userEntry, const char* description, float value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %40s",description);
	pcl::console::print_value(" %5f ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntry(int userEntry, const char* description, const char* value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %40s",description);
	pcl::console::print_value(" %15s ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntry(int userEntry, const char* description)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);
	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %40s \n",description);
}

void Menue::printMenueEntryMid(int userEntry, const char* description, int value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %25s",description);
	pcl::console::print_value(" %5d ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntryMid(int userEntry, const char* description, float value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %25s",description);
	pcl::console::print_value(" %5f ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntryMid(int userEntry, const char* description, const char* value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %25s",description);
	pcl::console::print_value(" %15s ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntryMid(int userEntry, const char* description)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);
	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %25s \n",description);
}

void Menue::printMenueEntrySmall(int userEntry, const char* description, int value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %15s",description);
	pcl::console::print_value(" %5d ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntrySmall(int userEntry, const char* description, const char* value, const char* end)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);

	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %15s",description);
	pcl::console::print_value(" %15s ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printMenueEntrySmall(int userEntry, const char* description)
{
	char s[6];
	sprintf(s,"[%d]",userEntry);
	pcl::console::print_color(stdout,2,2,"%5s" ,s);
	pcl::console::print_info("  %15s \n",description);
}

void Menue::printOutput(const char* description, int value, const char* end)
{
	pcl::console::print_info("%s",description);
	pcl::console::print_value(" %d ",value);
	pcl::console::print_info("%s \n", end);
}
void Menue::printOutput(const char* description, const char* value, const char* end)
{
	pcl::console::print_info("%s",description);
	pcl::console::print_value(" %s ",value);
	pcl::console::print_info("%s \n", end);
}


void Menue::readValue(int* changedVar)
{
	int input;
	std::cin>>input;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
	*changedVar=input;
}
void Menue::readValue(bool* changedVar)
{
	bool input;
	std::cin>>input;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
	*changedVar=input;
}
void Menue::readValue(double* changedVar)
{
	double input;
	std::cin>>input;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
	*changedVar=input;
}
void Menue::readValue(float* changedVar)
{
	float input;
	std::cin>>input;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
	*changedVar=input;
}