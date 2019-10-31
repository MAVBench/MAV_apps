/*
 * datacontainer.cpp
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */

using namespace std;


#include <datacontainer.h>
#include <algorithm>

//constructor
DataContainer::DataContainer() {
	// TODO Auto-generated constructor stub

}


// setters
void DataContainer::setStatsAndClear(){
	for (std::vector<Data>::iterator it=container.begin(); it != container.end(); it++){
//	for (auto &el: container){
		it->setStatsAndClear();
	}
}


std::vector<Data>::iterator DataContainer::find_if_name_equal(std::string name){
	for (auto it = this->container.begin(); it !=this->container.end(); it++) {
		if (it->data_key_name == name){
			return it;
		}
	}
	return this->container.end();
}

void DataContainer::capture(std::string name, std::string mode, double data_value) {
	/*
	auto data_name_equal = [=](Data data_1){return data_1.data_key_name == name;};
	//find or allocate
	auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	*/
	auto it = find_if_name_equal(name);
	if (it == this->container.end()){
		if (mode == "end") {
			string exception("data with the name of " + name + "doesn't exist");
			cout<< "data with the name of " + name + " doesn't exist" <<endl;
			throw;
		}

		this->container.push_back(Data(name));
		it = this->container.end() - 1;
	}

	it->capture(data_value, mode);
}


bool DataContainer::data_name_equal(Data& data1){
	bool result = (data1.data_key_name == "point_cloud_deserialization");
	return result;
}


#ifdef ROS
void DataContainer::capture(std::string name, std::string mode, ros::Time data_value) {

	//auto data_name_equal = [=](Data &data_1){return (data_1.data_key_name == name);};
	//auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	auto it = find_if_name_equal(name);
	if (it == this->container.end()){
		if (mode == "end") {
			string exception("data with the name of " + name + "doesn't exist");
			cout<< "data with the name of " + name + " doesn't exist" <<endl;
			throw;
		}

		this->container.push_back(Data(name));
		it = this->container.end() - 1;
	}
	it->capture(data_value, mode);

/*

	//find or allocate
	bool reached_end = true;
	for (auto it = this->container.begin(); it !=this->container.end(); it++) {
		if (it->data_key_name == name){
			reached_end = false;
			it->capture(data_value, mode);
			break;
		}
	}
	if (reached_end){
		edthis->container.push_back(Data(name));
		auto it = this->container.end() - 1;
		it->capture(data_value, mode);
	}
*/
}
#endif

// getters
string DataContainer::getStatsInString(vector<string> stats_name, string leading_spaces){
	string results = "";
	for (vector<Data>::iterator it = this->container.begin(); it != this->container.end(); it++) {
		results += leading_spaces + "\"" + it->data_key_name + " stats are\":" + "\n";
		results += it->getStatsInString(stats_name, leading_spaces + "\t");
		results += "\n";
	}
	return results;
}

void DataContainer::findDataByName(string name, Data **data){
	//auto data_name_equal = [=](Data data_1){return data_1.data_key_name == name;};
	//auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	auto it = find_if_name_equal(name);

	*data = (it == this->container.end() ? nullptr: &(*it));
}

Data* DataContainer::findDataByName(string name) {
	//auto data_name_equal = [=](Data data_1){return data_1.data_key_name == name;};
	//auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	auto it = find_if_name_equal(name);
	if (it == this->container.end()){
		return nullptr;
	}else{
		return &(*it);
	}
}



/*
// streams
template <class T, class S>
void printDataLastValue(string name) {

}
*/

DataContainer::~DataContainer() {
	// TODO Auto-generated destructor stub
}
/*
template class DataContainer<double, double>;
#ifdef ROS
template class DataContainer<ros::Time, ros::Duration>;
#endif
*/
