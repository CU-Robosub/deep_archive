#include <fstream>
#include <iostream>
#include <C:\Users\Michael\Desktop\json.hpp>
#include <string>

using namespace std;

/*
This is the .cpp class. It is made to read a .json file and retrieve values from it.

It can also edit external json files, add new values to keys, and add new key-value pairs altogether
*/

class ConfigParser {

	//this data type is from json.hpp
	nlohmann::json jsonstore;

	public:
		ConfigParser(char* filename);
		string get_category(char* categoryname);
		void write_to_file(char* filename);
		void setting_change(string setting_key_name, char* new_value);
};


/*
The constructor. Each new instance of ConfigParser reads in the source from one singular external JSON file, stores the file contents, and closes the file.  
*/

ConfigParser::ConfigParser(char* filename) {
	ifstream configFile(filename);
	configFile >> jsonstore;
	configFile.close();
}


/*The config file should only have strings as the values. So anything that could be an
integer or a boolean should be expressed inside quotation marks "" in both the .cpp and the .json
*/

string ConfigParser::get_category(char* categoryname) {
	return (jsonstore.at(categoryname).get<std::string>());
}


/*

This function can either:

*If the key name in the first parameter already exists in the JSON) Change the value of an existing key in the JSON

*If the key name does not exist in the JSON) Make a new key-value pair in the JSON 

*/

void ConfigParser::setting_change(string setting_key_name, char* new_value) {
	jsonstore[setting_key_name] = new_value;
}



/*
This write function is made to rewrite the JSON setting files, for both configs and pub/sub.

Use this function after you have used the function ... to update the external the JSON file.

MAKE SURE THE FILENAME FOR THE JSON FILE IS WHAT YOU WANT IT TO BE (the original JSON settings file name).

I suggest using the same file name as the original input JSON file's name if it is necessary to change multiple files
*/

void ConfigParser::write_to_file(char* filename) {
	ofstream external_file(filename);
	external_file << setw(4) << jsonstore << endl;
	external_file.close();
}


/*
This is an example of how the files and class may work.
*/

/*
int main() {
	ConfigParser x = ConfigParser("unrealConfig.json");
	x.setting_change("pi", "three point one four");
	x.write_to_file("external_test_file.json");
}
*/
