#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>
#include <iostream>
#include "zpipe.h"

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <memory>

using boost::asio::ip::tcp;

using namespace std;

class MyGrabber : public pcl::OpenNIGrabber
{
		public:
				boost::shared_ptr< pcl::PointCloud < pcl::PointXYZ > >
						convertToXYZPointCloud
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

class SimpleOpenNIViewer
{
		public:
				void GetPixelBuffer
						(
						 const XnDepthPixel *pixels,
						 unsigned pixelLen,
						 unsigned & dataLen
						);

				void EncodeDepthImage
						(
						 const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage,
						 int encodeParam,
						 unsigned & compressedLength
						);

				void DecodePixels
						(
						 unsigned char *compressed,
						 unsigned length,
						 XnDepthPixel *decompressed,
						 unsigned imgWidth,
						 unsigned imgHeight
						);

				int GetMeanValue
						(
						 const XnDepthPixel *pixels,
						 int rows,
						 int cols, 
						 int index,
						 int b
						);

				void WriteLUT();

				void ComputeFPS();

				SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer")
		{
				port = 11117;
				wroteLut = false;
				for(int i =0; i <640*480;i++)
						lut[i] = -1;
				socket = NULL;
				encodeParam = -1;
		}

				const static unsigned outLen = 640*480*5;	
				unsigned char out[outLen];
				int encodeParam;

				pcl::visualization::CloudViewer viewer;
				//convertToXYZRGBPointCloud
				unsigned short pixelCopy[640*480*2];
				bool wroteLut;
				bool turn;
				int lut[640*480];
				tcp::socket *socket;
				int counter;
				int port;
				double start_time;
				double frames_per_second;
				MyGrabber* grabber;

				void cloud_callback_  (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
				{
						//if (!viewer.wasStopped())
						//	viewer.showCloud (cloud);
				}

				void cloud_cb_ (const boost::shared_ptr<openni_wrapper::Image>& rgbImage, const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage, float focalLength)
				{
						unsigned compressedLength;
						EncodeDepthImage(depthImage, encodeParam, compressedLength);

						if (socket != NULL && compressedLength > 0)
						{
								boost::asio::write (*socket, boost::asio::buffer (out, (compressedLength) * sizeof (unsigned char)));
						}
/*
						XnDepthPixel decompressed[640*480];//pixelLen];
						DecodePixels
								(
								 out+20,
								 compressedLength-20,
								 decompressed,
								 640,
								 480
								);*/
						/*
						   boost::shared_ptr< pcl::PointCloud < pcl::PointXYZ > > decompressedCloud =
						   grabber->convertToXYZPointCloud
						   (
						   decompressed,
						   pixelLen,
						   depthImage->getBaseline(),
						   focalLength,
						   depthImage->getShadowValue(),
						   depthImage->getNoSampleValue()
						   );

						   if (!viewer.wasStopped())
						   viewer.showCloud (decompressedCloud);
						 */
						if(!wroteLut)
						{
								wroteLut=true;
								WriteLUT();
						}

						ComputeFPS();
				}

				void run ()
				{
						counter = 0;
						grabber = new MyGrabber();

						boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1, _2, _3);
						boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)> handler_function = boost::bind (&SimpleOpenNIViewer::cloud_callback_, this, _1);
						grabber->registerCallback (handler_function);

						grabber->registerCallback (f);

						boost::asio::io_service io_service;
						tcp::endpoint endpoint (tcp::v4 (), static_cast<unsigned short> (port));
						tcp::acceptor acceptor (io_service, endpoint);
							socket = new tcp::socket(io_service);

						std::cout << "Listening on port " << port << "..." << std::endl;
								acceptor.accept (*socket);
						double start_time = pcl::getTime ();

						std::cout << "Client connected." << std::endl;

						grabber->start ();

						while (!viewer.wasStopped())
						{
								// boost::this_thread::sleep (boost::posix_time::seconds (1));
						}

						grabber->stop ();
						delete socket;
				}

};

void SimpleOpenNIViewer::WriteLUT()
{
		FILE * pFile;
		pFile = fopen ("lut", "wb");
		fwrite (lut , sizeof(int), sizeof(lut)/sizeof(int), pFile);
		fclose (pFile);
}

void SimpleOpenNIViewer::ComputeFPS()
{
		counter++;

		double new_time = pcl::getTime ();
		double elapsed_time = new_time - start_time;
		if (elapsed_time > 1.0)
		{
				frames_per_second = counter / elapsed_time;
				start_time = new_time;
				counter = 0;
		}

		std::cout << "fps: " << frames_per_second << std::endl;
}

void SimpleOpenNIViewer::EncodeDepthImage
(
	const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage,
	int encodeParam,
	unsigned & compressedLength
)
{
		const xn::DepthMetaData & metaData = depthImage->getDepthMetaData();
		const XnDepthMetaData* xnMetaData = metaData.GetUnderlying();
		const XnDepthPixel *pixels = xnMetaData->pData;
		unsigned pixelLen = depthImage->getWidth()*depthImage->getHeight();

		unsigned dataLen;//Length of the pixel buffer to compress.
		GetPixelBuffer(pixels, pixelLen, dataLen);

		typedef unsigned char * byte_ptr;
		//To send floats, convert them to character arrays them add them to the main array.
		byte_ptr pixelChars = (unsigned char*)pixelCopy;
		float baseline = depthImage->getBaseline();
		float shadowVal = depthImage->getShadowValue();
		float noSampleVal = depthImage->getNoSampleValue();
		float focalLength = depthImage->getFocalLength();
		byte_ptr baselineC = (byte_ptr)(&(baseline));
		byte_ptr focalLengthC = (byte_ptr)(&(focalLength));
		byte_ptr shadowValC = (byte_ptr)(&(shadowVal));
		byte_ptr noSampleValC = (byte_ptr)(&(noSampleVal));

		deflateKinectData(pixelChars,dataLen,out+20, outLen, encodeParam, compressedLength);
		byte_ptr compressedLenC = (byte_ptr)(&(compressedLength));

		for(int i=0;i<4;i++)
		{
				out[i] = baselineC[i];
				out[i+4] = focalLengthC[i];
				out[i+8] = shadowValC[i];
				out[i+12] = noSampleValC[i];
				out[i+16] = compressedLenC[i];
		}

		cout<<endl;

		cout<<"Original Len: "<<dataLen<<endl;
		cout<<"CompressedLen: "<<compressedLength<<endl;
		cout<<"Compression Ratio: "<<(((double)dataLen)/compressedLength)<<endl;
		cout<<"Total Compression Ratio: "<<(((double)pixelLen*2)/compressedLength)<<endl;

		compressedLength+=20;
}

void SimpleOpenNIViewer::GetPixelBuffer(const XnDepthPixel *pixels, unsigned pixelLen, unsigned & dataLen)
{
		if(pixelLen<=0 || pixels == NULL)
		{
				return;
		}

		unsigned a = 500;
		dataLen = 0;//100*100*2;
		int b = a*(a+1);
		int vals[9], val;
		float mean,diff;
		int X;
		for(int i=0;i <pixelLen;i++)
		{
				X = pixels[i];			//== 0 ? 0 : (a*(a+1))/pixels[i];
				int r = i/640;
				int c = i%640;
				if(r<160 || r>=320 || c<213 || c>426)
				{
						if(r%3==1 && c%3==1)
						{
								X = GetMeanValue
										(
										 pixels,
										 480,
										 640,
										 i,
										 b
										);

								if(!wroteLut)
								{
										lut[i-641]=dataLen;
										lut[i-640]=dataLen;
										lut[i-639]=dataLen;
										lut[i-1] = dataLen;
										lut[i] = dataLen;
										lut[i+1] = dataLen;
										lut[i+639] = dataLen;
										lut[i+640] = dataLen;
										lut[i+641] = dataLen;
								}
						}
						else
								continue;
				}
				else
				{
						if(!wroteLut)
								lut[i] = dataLen;
				}

				pixelCopy[dataLen] = X;
				dataLen++;
		}

		dataLen*=2;
}

		int SimpleOpenNIViewer::GetMeanValue
(
 const XnDepthPixel *pixels,
 int rows,
 int cols, 
 int index,
 int b
 )
{
		if(pixels[index]==0)
		{
				return 0;
		}

		//Vals array pattern, where #4 is the current element.
		//0 1 2
		//3 4 5
		//6 7 8
		int vals[9];
		vals[0] = index < (cols+1) ? -1 : pixels[index-(cols+1)];
		vals[1] = index <  cols ? -1 : pixels[index-cols];
		vals[2] = index < (cols-1) ? -1 : pixels[index-(cols-1)];

		vals[3] = index <1   ? -1 : pixels[index-1];
		vals[4] = pixels[index];
		vals[5] = index == cols*rows-1 ? -1 : pixels[index+1];

		vals[6] = index > cols*(rows-1) ? -1 : pixels[index+(cols-1)];
		vals[7] = index > cols*(rows-1)-1 ? -1 : pixels[index+cols];
		vals[8] = index > cols*(rows-1)-2 ? -1 : pixels[index+(cols+1)];

		float mean = 0;
		float num = 0;
		for(int i=0; i<9; i++)
		{
				if(vals[i]!=-1)
				{
						mean+=vals[i];
						num++;
				}
		}

		mean/=num;

		int val = 0;
		float diff = -1;
		for(int i=0; i<9; i++)
		{
				float valDiff = abs(vals[i]-mean);
				if(vals[i]!=-1 && (diff==-1 || valDiff<diff))
				{
						diff = valDiff;
						val = vals[i];
				}
		}

		if(val>0)
		{
				val = b/val;
				val = b/val;
		}

		return val; 
}

		void SimpleOpenNIViewer::DecodePixels
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
		//inflateKinectData(compressed+16, length, decompressedChars, imgSize*2, decompressedLen);
		inflateKinectData(compressed, length, decompressedChars, imgSize*2, decompressedLen);
		cout<<"Decompressed Length: "<<decompressedLen<<" Predicated: "<<(imgSize*2)<<endl;
		for(int j=0;j <imgSize;j++)
		{
				decompressed[j] = lut[j]==-1?0:decomShort[lut[j]];
		}

		cout<<"Done decoding!"<<endl;
}

int main ()
{
		SimpleOpenNIViewer v;
		v.run ();
		return 0;
}
