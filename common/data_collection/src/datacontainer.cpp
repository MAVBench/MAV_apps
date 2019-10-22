/*
 * datacontainer.cpp
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */

#include <datacontainer.h>
template <class T, class S>
DataContainer<T, S>::DataContainer() {
	// TODO Auto-generated constructor stub

}


template <class T, class S>
void DataContainer<T, S>::setStats(){
	for (auto &el:container){
		el.setStats();
	}
}

template <class T, class S>
void DataContainer<T, S>::capture(string name, string mode, T data_value) {
	auto data_name_equal = [=](Data<T, S> data_1){return data_1.data_key_name == name;};

	//find or allocate
	auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	if (it == this->container.end()){
		if (mode == "end") {
			string exception("data with the name of " + name + "doesn't exist");
			cout<< "data with the name of " + name + " doesn't exist" <<endl;
			throw;
		}

		Data<T, S> new_data(name);
		this->container.push_back(new_data);
		it = this->container.end() - 1;
	}

	it->capture(data_value, mode);
}




template <class T, class S>
void DataContainer<T, S>::findDataByName(string name, Data<T, S> **data){
	auto data_name_equal = [=](Data<T, S> data_1){return data_1.data_key_name == name;};
	auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
	*data = (it == this->container.end() ? nullptr: &(*it));
}

template <class T, class S>
Data<T, S>* DataContainer<T, S>::findDataByName(string name) {
	auto data_name_equal = [=](Data<T, S> data_1){return data_1.data_key_name == name;};
	auto it = std::find_if(this->container.begin(), this->container.end(), data_name_equal);
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

template <class T, class S>
DataContainer<T, S>::~DataContainer() {
	// TODO Auto-generated destructor stub
}

template class DataContainer<float, float>;
template class DataContainer<ros::Time, ros::Duration>;
