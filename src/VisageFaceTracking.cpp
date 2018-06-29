/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 3* contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * 
 *
 * @author Joe Bedard <bedard@in.tum.de>
 */


#include <string>
#include <list>
#include <iostream>
#include <iomanip>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <opencv/cv.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>

#include <visageVision.h>

#define _USE_MATH_DEFINES
#include <math.h>

//namespace VisageSDK {
//   void __declspec(dllimport) initializeLicenseManager(const char *licenseKeyFileFolder);
//}

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.VisageFaceTracking" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Measurement::ImageMeasurement.
 *
 * @par Configuration
 * The configuration tag contains a \c <dsvl_input> configuration.
 * For details, see the DirectShow documentation...
 *
 */
class VisageFaceTracking
	: public Dataflow::Component
{
public:

	/** constructor */
	VisageFaceTracking( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~VisageFaceTracking();

	/** starts the camera */
	void start();

	/** stops the camera */
	void stop();

protected:
	
	// the ports
	Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

	Dataflow::PushSupplier< Measurement::Pose > m_outPort;
	
	void newImage(Measurement::ImageMeasurement image);

private:

	int * track_stat;

	VisageSDK::VisageTracker * m_Tracker = 0;

   // convert from ubitrack enum to visage enum (for image pixel format)
   int switchPixelFormat(Vision::Image::PixelFormat pf);
};


VisageFaceTracking::VisageFaceTracking( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )	
	, m_outPort( "Output", *this )
	, m_inPort( "ImageInput", *this, boost::bind(&VisageFaceTracking::newImage, this, _1))
{
   if (subgraph->m_DataflowAttributes.hasAttribute("configurationFile")) 
   {
      std::string configurationFile = subgraph->m_DataflowAttributes.getAttributeString("configurationFile");
	   m_Tracker = new VisageSDK::VisageTracker(configurationFile.c_str());
   }
   else 
   {
      std::ostringstream os;
      os << "Visage Configuration File is required, but was not provided!";
      UBITRACK_THROW(os.str());
   }
   //VisageSDK::initializeLicenseManager("C:\\Program Files\\Visage Technologies");
}


int VisageFaceTracking::switchPixelFormat(Vision::Image::PixelFormat pf)
{
   using Ubitrack::Vision::Image;
   switch (pf)
   {
   case Image::LUMINANCE:
      return VISAGE_FRAMEGRABBER_FMT_LUMINANCE;
   case Image::RGB:
      return VISAGE_FRAMEGRABBER_FMT_RGB;
   case Image::BGR:
      return VISAGE_FRAMEGRABBER_FMT_BGR;
   case Image::RGBA:
      return VISAGE_FRAMEGRABBER_FMT_RGBA;
   case Image::BGRA:
      return VISAGE_FRAMEGRABBER_FMT_BGRA;
   case Image::YUV422:
   case Image::YUV411:
   case Image::RAW:
   case Image::DEPTH:
   case Image::UNKNOWN_PIXELFORMAT:
   default:
      return -1;
   }
}


void VisageFaceTracking::newImage(Measurement::ImageMeasurement image) 
{
   int visageFormat = switchPixelFormat(image->pixelFormat());
   if (visageFormat == -1)
   {
      LOG4CPP_ERROR(logger, "YUV422, YUV411, RAW, DEPTH, UNKNOWN_PIXELFORMAT are not supported by Visage");
      LOG4CPP_ERROR(logger, "image->pixelFormat() == " << image->pixelFormat());
   }
   else
   {
      // the direct show image is rotated 180 degrees
      cv::Mat dest;
      cv::rotate(image->Mat(), dest, cv::RotateFlags::ROTATE_180);

      // pass the image to Visage
	   const char * data = (char *)dest.data;
   	VisageSDK::FaceData faceData;
	   track_stat = m_Tracker->track(image->width(), image->height(), data, &faceData, visageFormat, VISAGE_FRAMEGRABBER_ORIGIN_TL);
      
      if (faceData.trackingQuality >= 0.1f)
      {
         LOG4CPP_DEBUG(logger, "Head Rotation X Y Z:  " << faceData.faceRotation[0] << " " << faceData.faceRotation[1] << " " << faceData.faceRotation[2]);
         LOG4CPP_DEBUG(logger, "Head Translation X Y Z: " << faceData.faceTranslation[0] << " " << faceData.faceTranslation[1] << " " << faceData.faceTranslation[2]);
         LOG4CPP_DEBUG(logger, "Tracking Quality: " << faceData.trackingQuality);
      }
     
     // output head pose
      Math::Quaternion headRot = Math::Quaternion(-faceData.faceRotation[2], -faceData.faceRotation[1], faceData.faceRotation[0]);
      Math::Vector3d headTrans = Math::Vector3d(faceData.faceTranslation[0], faceData.faceTranslation[1], -faceData.faceTranslation[2]);
      Math::Pose headPose = Math::Pose(headRot, headTrans);
      Math::Pose headPose2 = Math::Pose(headRot, headTrans);
      Measurement::Pose meaHeadPose = Measurement::Pose(image.time(), headPose);
      m_outPort.send(meaHeadPose);
   }
}


VisageFaceTracking::~VisageFaceTracking()
{
	track_stat = m_Tracker->track(0, 0, 0, 0);
	delete m_Tracker;
}


void VisageFaceTracking::start()
{
	
	Component::start();
}


void VisageFaceTracking::stop()
{
	
	Component::stop();
}


} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::VisageFaceTracking > ( "VisageFaceTracking" );
}

