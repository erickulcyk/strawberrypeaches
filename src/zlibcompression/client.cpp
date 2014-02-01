#include <boost/asio.hpp>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <memory>
#include "zpipe.h"

using namespace std;
using boost::asio::ip::tcp;

class MyGrabber : public pcl::OpenNIGrabber
{
		public:
				boost::shared_ptr< pcl::PointCloud < pcl::PointXYZ > > convertToXYZPointCloud
						(
						 unsigned short* darray,
						 unsigned dLength,
						 float baseline,
						 float focal_length,
						 float shadow_value,
						 float no_sample_value
						)
						{
								boost::shared_ptr<xn::DepthMetaData> dmd_ptr (new xn::DepthMetaData);
								dmd_ptr->ReAdjust(640,480,darray);

								boost::shared_ptr<openni_wrapper::DepthImage> di_ptr (
												new openni_wrapper::DepthImage (dmd_ptr, baseline, focal_length, shadow_value, no_sample_value));

								return pcl::OpenNIGrabber::convertToXYZPointCloud(di_ptr);
						}
};

int * lut;

void readLUT()
{
		FILE * pFile;
		long lSize;
		size_t result;

		pFile = fopen ( "lut" , "rb" );
		if (pFile==NULL) {fputs ("File error",stderr); exit (1);}

		// obtain file size:
		fseek (pFile , 0 , SEEK_END);
		lSize = ftell (pFile);
		rewind (pFile);

		cout<<"Found Size: "<<lSize<<endl;
		// allocate memory to contain the whole file:
		lut = (int*) malloc (sizeof(char)*lSize);
		if (lut == NULL) {fputs ("Memory error",stderr); exit (2);}

		// copy the file into the buffer:
		result = fread (lut,sizeof(int),lSize/sizeof(int),pFile);
		if (result*sizeof(int) != lSize) {cout<<" Did not get the right size.  Got: "<<result<<endl;exit (3);}

		/* the whole file is now loaded in the memory buffer. */

		// terminate
		fclose (pFile);
		//free (buffer);
}

class PCLStreamClient
{
		void DecodePixels
				(
				 unsigned char *compressed,
				 unsigned length,
				 XnDepthPixel *decompressed,
				 unsigned imgWidth,
				 unsigned imgHeight
				);

		int port;
		string ipAddress;

		public:
		PCLStreamClient()
		{
				port = 11117;
				ipAddress = "127.0.0.1";
		}

		void run()
		{
				try
				{
						boost::asio::io_service io_service;	
						tcp::resolver resolver(io_service);
						tcp::resolver::query query(ipAddress, "11117");
						tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
						tcp::resolver::iterator end;
						tcp::socket socket(io_service);
						boost::asio::connect(socket, resolver.resolve(query));


						MyGrabber *grabber = new MyGrabber();
						pcl::visualization::CloudViewer viewer("My Client");
						/*boost::system::error_code error = boost::asio::error::host_not_found;
						  while (error && endpoint_iterator != end)
						  {
						  cout<<"Got error"<<endl;
						  socket.close();
						  socket.connect(*endpoint_iterator++, error);
						  }

						  if (error)
						  {
						  cout<<"Got error 2"<<endl;
						  throw boost::system::system_error(error);
						  }*/
						for (;;)
						{
								boost::array<float, 4> buf;
								boost::array<int, 1> buf2;
								boost::system::error_code error;

								size_t len = socket.read_some(boost::asio::buffer(buf), error);
								if (error == boost::asio::error::eof)
										break; // Connection closed cleanly by peer.
								else if (error)
										throw boost::system::system_error(error); // Some other error.
								cout<<"Just read in: "<<len<<" bytes"<<endl;

								len = socket.read_some(boost::asio::buffer(buf2), error);
								if (error == boost::asio::error::eof)
										break; // Connection closed cleanly by peer.
								else if (error)
										throw boost::system::system_error(error); // Some other error.
								cout<<"Just read in2: "<<len<<" bytes"<<endl;

								boost::array<unsigned short, 640*480> buf3;
								len = socket.read_some(boost::asio::buffer(buf3), error);
								if (error == boost::asio::error::eof)
										break; // Connection closed cleanly by peer.
								else if (error)
										throw boost::system::system_error(error); // Some other error.
								cout<<"Just read in3: "<<len<<" bytes"<<endl;
								//std::cout.write(buf.data(), len);
								cout<<buf[0]<<" "<<buf[1]<<" "<<buf[2]<<" "<<buf[3]<<" "<<buf2[0]<<endl;

								XnDepthPixel decompressed[640*480];
								DecodePixels
										(
										 (unsigned char *)buf3.data(),
										 buf2[0],
										 decompressed,
										 640,
										 480
										);

								boost::shared_ptr< pcl::PointCloud < pcl::PointXYZ > > decompressedCloud =
										grabber->convertToXYZPointCloud
										(
										 decompressed,
										 buf2[0],
										 buf[0],
										 buf[1],
										 buf[2],
										 buf[3]
										);

								if (!viewer.wasStopped())
										viewer.showCloud (decompressedCloud);

						}
				}
				catch (std::exception& e)
				{
						cout<<"Caught Exception :("<<endl;
						std::cerr << e.what() << std::endl;
				}
		}
};

		void PCLStreamClient::DecodePixels
(
 unsigned char *compressed,
 unsigned length,
 XnDepthPixel *decompressed,
 unsigned imgWidth,
 unsigned imgHeight
 )
{
		if(compressed == NULL || decompressed == NULL || length <= 0 || imgWidth <= 0 || imgHeight <= 0 )
		{
				return;
		}

		unsigned imgSize = imgWidth*imgHeight;

		unsigned char decompressedChars[imgSize*2];// = (unsigned char *) decompressed;
		unsigned short *decomShort = (unsigned short *)decompressedChars;
		unsigned decompressedLen;
		inflateKinectData(compressed, length, decompressedChars, imgSize*2, decompressedLen);
		cout<<"Decompressed Length: "<<decompressedLen<<" Predicated: "<<(imgSize*2)<<endl;
		for(int j=0;j <imgSize;j++)
		{
				decompressed[j] = lut[j]==-1?0:decomShort[lut[j]];
		}
}


int main()
{
		readLUT();
		PCLStreamClient client;
		client.run();
}
