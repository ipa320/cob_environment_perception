#pragma once

class Debug_GPX {
	std::ofstream file_gpx_;
	
public:

	Debug_GPX() {
	}
	Debug_GPX(const std::string &fn) {
		open(fn);
	}

	~Debug_GPX()
	{
	  file_gpx_<< "</trkseg></trk></gpx>";
	  file_gpx_.close();
	}
	
	void open(const std::string &fn) {
		file_gpx_.open(fn.c_str());
		file_gpx_ << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\r\n<gpx xmlns=\"http://www.topografix.com/GPX/1/1\" version=\"1.1\" creator=\"Wikipedia\"\r\n    xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n    xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\"><trk><name>Trackname2</name><trkseg>";
	}
	
	void add_pt(const float lat, const float lon) {
	  file_gpx_ << "<trkpt lat=\""<<lat<<"\" lon=\""<<lon<<"\">\r\n  <time>2011-01-15T23:59:01Z</time></trkpt>";
	  file_gpx_.flush();
  }
};
