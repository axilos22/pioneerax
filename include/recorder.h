/** @author Axel J. for ECN
  */
#ifndef RECORDER_H
#define RECORDER_H
//cpp
#include <iostream>
#include <istream>
#include <fstream>
#include <vector>

class Recorder
{
	public:
		/*param*/
		std::string m_filePath;
		/*Functions*/
		Recorder(std::string filePath);
		void setColumns(std::string colNames);
		void addRecord(std::vector<std::string> data);
		~Recorder();
	};

#endif
