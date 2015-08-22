#pragma once
#include <iostream> 
#include <sstream>  
class Menue
{
public:
	Menue(void);
	~Menue(void);
	static void Menue::printMenueEntry(int userEntry, const char* description, int value, const char* end="");
	static void Menue::printMenueEntry(int userEntry, const char* description, float value, const char* end="");
	static void Menue::printMenueEntry(int userEntry, const char* description, const char* value, const char* end="");
	static void Menue::printMenueEntry(int userEntry, const char* description);

	static void Menue::printMenueEntryMid(int userEntry, const char* description, int value, const char* end="");
	static void Menue::printMenueEntryMid(int userEntry, const char* description, float value, const char* end="");
	static void Menue::printMenueEntryMid(int userEntry, const char* description, const char* value, const char* end="");
	static void Menue::printMenueEntryMid(int userEntry, const char* description);

	static void Menue::printMenueEntrySmall(int userEntry, const char* description, int value, const char* end="");
	static void Menue::printMenueEntrySmall(int userEntry, const char* description, const char* value, const char* end="");
	static void Menue::printMenueEntrySmall(int userEntry, const char* description);
	
	static void Menue::printOutput(const char* description, int value, const char* end="");
	static void Menue::printOutput(const char* description, const char* value, const char* end="");

	static void Menue::readValue(float* changedVar);
	static void Menue::readValue(double* changedVar);
	static void Menue::readValue(bool* changedVar);
	static void Menue::readValue(int* changedVar);
};

