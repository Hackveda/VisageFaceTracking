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
#include <utDataflow/PullConsumer.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <opencv/cv.h>
#include <utVision/Image.h>


#include <utAlgorithm/Homography.h>
#include <utAlgorithm/PoseEstimation2D3D/PlanarPoseEstimation.h>
#include <utAlgorithm/Projection.h>

#define DO_TIMING
#ifdef DO_TIMING
#include <utUtil/BlockTimer.h>
#endif
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

	class VisageFaceIPDEstimation
		: public Dataflow::TriggerComponent
	{
	public:
		VisageFaceIPDEstimation(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
			: Dataflow::TriggerComponent(sName, pConfig)
			, m_inPortImageA("ImageA", *this)
			, m_inPortImageB("ImageB", *this)
			, m_inPortAtoB("AB", *this)
			, m_outPortIPD("IPD", *this)
		{
				
				std::string pathA = pConfig->m_DataflowAttributes.getAttributeString("configurationFileA");
				
				std::string pathB = pConfig->m_DataflowAttributes.getAttributeString("configurationFileB");
				

				m_tracker[0].reset(new VisageSDK::VisageTracker(pathA.c_str()));
				m_tracker[1].reset(new VisageSDK::VisageTracker(pathB.c_str()));

		}

		void compute(Measurement::Timestamp t)
		{
			m_images[0].push_back(m_inPortImageA.get());
			m_images[1].push_back(m_inPortImageB.get());
			m_posesAtoB.push_back(m_inPortAtoB.get());

			if (m_images[0].size() < m_minImages)
				return;

			
			double ipdDelta = (m_maxIPD - m_minIPD) / m_steps;

			std::vector<double> errorValue;
			std::vector<double> ipdValue;

			for (int i = 0; i < m_steps; i++) {
				double ipd = m_minIPD + ipdDelta*i;
				double error = calculateMeanErrorForIPD(ipd);
				if (error>0){
					ipdValue.push_back(ipd);
					errorValue.push_back(error);
					LOG4CPP_INFO(logger, "IPD : ERROR  " << ipd << " : " << error);
				}
			}

			int minErrorIndex;
			double minError = std::numeric_limits<double>::max();
			for (int i = 0; i < m_steps; i++) {
				//LOG4CPP_INFO(logger, "IPD : ERROR  " << ipdValue[i] << " : " << errorValue[i]);

			}

		}
	private:

		double calculateMeanErrorForIPD(double ipd){
			cv::Mat dest;
			

			std::vector<Math::Pose> headPoses[2];
			std::vector<Math::Pose> a2bPoses;

			for (int i = 0; i < m_images[0].size(); i++){

				bool isImageOK[2] = { false, false };
				Math::Pose headPose[2];
				

				for (int j = 0; j < 2; j++){
					Measurement::ImageMeasurement image = m_images[j][i];

					if (image->origin()) {
						cv::rotate(image->Mat(), dest, cv::RotateFlags::ROTATE_180);
					}
					else {
						dest = image->Mat();
					}
					
					m_tracker[j]->setIPD(ipd);
					m_tracker[j]->reset();

					const char * data = (char *)dest.data;
					VisageSDK::FaceData faceData;
					int* track_stat;
					track_stat =m_tracker[j]->track(image->width(), image->height(), data, &faceData, VISAGE_FRAMEGRABBER_FMT_RGB, 0,0, image.time(),1);

					if (track_stat && track_stat[0] == TRACK_STAT_OK && faceData.trackingQuality >= 0.6f)
					{
				

						//Math::Quaternion headRot = Math::Quaternion(-faceData.faceRotation[2], faceData.faceRotation[1], -faceData.faceRotation[0]);
						Math::Quaternion headRot;
						//Math::Vector3d headTrans = Math::Vector3d(faceData.faceTranslation[0], faceData.faceTranslation[1], faceData.faceTranslation[2]);
						Math::Vector3d headTrans;
						if (image->origin()) {
							headTrans = Math::Vector3d(faceData.faceTranslation[0], faceData.faceTranslation[1], -faceData.faceTranslation[2]);
							headRot = Math::Quaternion(-faceData.faceRotation[2], -faceData.faceRotation[1], faceData.faceRotation[0]);
						}
						else {
							headTrans = Math::Vector3d(-faceData.faceTranslation[0], faceData.faceTranslation[1], -faceData.faceTranslation[2]);
							headRot = Math::Quaternion(faceData.faceRotation[2], faceData.faceRotation[1], faceData.faceRotation[0]);
						}
						headPose[j] = Math::Pose(headRot, headTrans);

						//LOG4CPP_INFO(logger, "Tracking IPD : ERROR  " << m_tracker[j]->getIPD() << " : " << headPose << " index: " << j << " : " << i);

						
						isImageOK[j] = true;
					}

				}


				if (isImageOK[0] && isImageOK[1]){
					headPoses[0].push_back(headPose[0]);
					headPoses[1].push_back(headPose[1]);
					a2bPoses.push_back(*m_posesAtoB[i]);
				}

				

				


			}

			if (headPoses[0].size() == 0)
				return -1;

			double result = 0;

			for (int i = 0; i < headPoses[0].size(); i++)
			{
				Math::Pose a2b = a2bPoses[i];
				Math::Pose a2Head_fromB = a2b *  headPoses[1][i];
				double distance = boost::numeric::ublas::norm_2(headPoses[0][i].translation() - a2Head_fromB.translation());
				//LOG4CPP_INFO(logger, "A2Head : A_fromB2Head : A2B : B2Head   " << headPoses[0][i] << " : " << a2Head_fromB << " : " << a2b << " : " << headPoses[1][i]);
				result = distance*distance;
			}

			result = result / headPoses[0].size();
			result = std::sqrt(result);
			return result;
		}


		/** Input port A of the component. */
		Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inPortImageA;
		Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inPortImageB;

		/** Input port B of the component. */
		Dataflow::TriggerInPort< Measurement::Pose> m_inPortAtoB;

		/** Output port of the component. */
		Dataflow::TriggerOutPort< Measurement::Distance > m_outPortIPD;

		std::vector<Measurement::ImageMeasurement> m_images[2];
		//std::vector<Measurement::ImageMeasurement> m_imagesB;
		std::vector<Measurement::Pose> m_posesAtoB;
		boost::shared_ptr<VisageSDK::VisageTracker> m_tracker[2];
		//boost::shared_ptr<VisageSDK::VisageTracker> m_trackerB;

		double m_minIPD = 0.055; // 0.051;
		double m_maxIPD = 0.075;// 0.077;
		int m_steps = 100;
		int m_minImages = 2;
	};
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

	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inPortIntrinsics;

	Dataflow::PushSupplier< Measurement::Pose > m_outPort;

	Dataflow::PushSupplier< Measurement::ErrorPose > m_outPortError;
	
	void newImage(Measurement::ImageMeasurement image);

private:

	int * track_stat;

	VisageSDK::VisageTracker * m_Tracker = 0;

   // convert from ubitrack enum to visage enum (for image pixel format)
   int switchPixelFormat(Vision::Image::PixelFormat pf);

   double m_covarScalePos[3];
   double m_covarScaleRot[3];
#ifdef DO_TIMING
   Ubitrack::Util::BlockTimer m_TimerAll;
   Ubitrack::Util::BlockTimer m_TimerTracking;

#endif
};


VisageFaceTracking::VisageFaceTracking( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )	
	, m_outPort( "Output", *this )
	, m_inPort( "ImageInput", *this, boost::bind(&VisageFaceTracking::newImage, this, _1))
	, m_inPortIntrinsics("Intrinsics", *this)
	, m_outPortError("OutputError", *this)
#ifdef DO_TIMING
    , m_TimerAll("Visage All", "Ubitrack.Timing")
	, m_TimerTracking("Visage tracking", "Ubitrack.Timing")
#endif
{
   if (subgraph->m_DataflowAttributes.hasAttribute("configurationFile")) 
   {
      std::string configurationFile = subgraph->m_DataflowAttributes.getAttributeString("configurationFile");
	   m_Tracker = new VisageSDK::VisageTracker(configurationFile.c_str());



	


	   std::string covarScalePosString;
	   if (subgraph->m_DataflowAttributes.hasAttribute("covarScalePos"))
		   covarScalePosString = subgraph->m_DataflowAttributes.getAttributeString("covarScalePos");
	   else
		   covarScalePosString = "1.0 1.0 1.0";

	   {
		   std::istringstream inStream(covarScalePosString);

		   inStream >> m_covarScalePos[0];
		   inStream >> m_covarScalePos[1];
		   inStream >> m_covarScalePos[2];

		   

	   }

	   std::string covarScaleRotString;
	   if (subgraph->m_DataflowAttributes.hasAttribute("covarScaleRot"))
		   covarScaleRotString = subgraph->m_DataflowAttributes.getAttributeString("covarScaleRot");
	   else
		   covarScaleRotString = "1.0 1.0 1.0";

	   {
		   std::istringstream inStream(covarScaleRotString);

		   inStream >> m_covarScaleRot[0];
		   inStream >> m_covarScaleRot[1];
		   inStream >> m_covarScaleRot[2];

	   }
   }
   else 
   {
      std::ostringstream os;
      os << "Visage Configuration File is required, but was not provided!";
      UBITRACK_THROW(os.str());
   }
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
	UBITRACK_TIME(m_TimerAll);
   int visageFormat = switchPixelFormat(image->pixelFormat());
   if (visageFormat == -1)
   {
      LOG4CPP_ERROR(logger, "YUV422, YUV411, RAW, DEPTH, UNKNOWN_PIXELFORMAT are not supported by Visage");
      LOG4CPP_ERROR(logger, "image->pixelFormat() == " << image->pixelFormat());
   }
   else
   {
      cv::Mat dest;
	  int origin = image->origin();
	  if (image->origin() == 0) {
		  dest = image->Mat();
	  }
	  else {
          // the input image is flipped vertically
		  cv::flip(image->Mat(), dest, 0);
		  LOG4CPP_WARN(logger, "Input image is flipped. Consider flipping in the driver to improve performance.");
	  }

      // pass the image to Visage
	  const char * data = (char *)dest.data;
   	  VisageSDK::FaceData faceData;
	  {
		  UBITRACK_TIME(m_TimerTracking);
		  track_stat = m_Tracker->track(image->width(), image->height(), data, &faceData, visageFormat, VISAGE_FRAMEGRABBER_ORIGIN_TL);		  
	  }
	  
      
      if (track_stat && track_stat[0] == TRACK_STAT_OK && faceData.trackingQuality >= 0.6f)
      {
		 //LOG4CPP_DEBUG(logger, "Tracking Quality: " << faceData.trackingQuality);
		 //LOG4CPP_DEBUG(logger, "Head Translation X Y Z: " << faceData.faceTranslation[0] << " " << faceData.faceTranslation[1] << " " << faceData.faceTranslation[2]);
		 //LOG4CPP_DEBUG(logger, "Head Rotation X Y Z:  " << faceData.faceRotation[0] << " " << faceData.faceRotation[1] << " " << faceData.faceRotation[2]);
		 
		 // output head pose
		 Math::Quaternion headRot = Math::Quaternion(faceData.faceRotation[2], faceData.faceRotation[1], faceData.faceRotation[0]);
		 Math::Quaternion headRotOrient = Math::Quaternion(0,1,0,0);
		 headRot = headRot * headRotOrient;
		 Math::Vector3d headTrans = Math::Vector3d(-faceData.faceTranslation[0], faceData.faceTranslation[1], -faceData.faceTranslation[2]);
		 Math::Pose headPose = Math::Pose(headRot, headTrans);
		 Measurement::Pose meaHeadPose = Measurement::Pose(image.time(), headPose);
		 m_outPort.send(meaHeadPose);
		 
		 
		 VisageSDK::FDP* fdp = faceData.featurePoints3DRelative;
			  
			   
		   const int indexFace[8]= { 2, 3,4,5,9,12,14,15};

		   std::vector<Math::Vector3d> p3D;
		   double originCorrection = -1.0;
		   if (origin) {
			   originCorrection = 1.0;
		   }
		   for (int i = 0; i < 8; i++){
			   int group = indexFace[i];
			   int groupSize = fdp->groupSize(group);
			   
			   for (int j = 0; j < groupSize; j++){
				   const VisageSDK::FeaturePoint& fp = fdp->getFP(group, j);
				   //LOG4CPP_INFO(logger, "FeaturePoint debug" << fp.quality);
				   //LOG4CPP_INFO(logger, "defined debug" << fp.defined);		   
				   
				   if (fp.defined  && fp.pos[0] != 0 && fp.pos[1] != 0 && fp.pos[2] != 0 && std::fabs(fp.pos[0]) < 1 && std::fabs(fp.pos[1]) < 1 && std::fabs(fp.pos[2]) < 1){
					   Math::Vector3d pos = Math::Vector3d(fp.pos[0], fp.pos[1], fp.pos[2]);
					   pos = headRotOrient * pos;
					   p3D.push_back(pos);
					   
				  }
			   }
		   }

		   Math::Matrix3x3d intrinsics = *m_inPortIntrinsics.get(image.time());
		   
		   /*LOG4CPP_INFO(logger, "draw debug");
		   cv::Mat debugImage = image->Mat();

		   Math::Matrix< double, 3, 4 > poseMat(headPose);
		   Math::Matrix3x4d projectionMatrix = boost::numeric::ublas::prod(intrinsics, poseMat);

		   for (int i = 0; i < p3D.size(); i++){
			   Math::Vector4d tmp(p3D[i][0], p3D[i][1], p3D[i][2], 1);
			   Math::Vector3d projectedPoint = boost::numeric::ublas::prod(projectionMatrix, tmp);

			   double wRef = projectedPoint[2];
			   projectedPoint = projectedPoint / wRef;

			   LOG4CPP_INFO(logger, "Point: " << p3D[i] << " : " << projectedPoint);
			   if (origin) {
				   cv::circle(debugImage, cv::Point2d(projectedPoint[0],projectedPoint[1]), 4, cv::Scalar(255, 0, 0), -1);
			   }
			   else {
				   cv::circle(debugImage, cv::Point2d(projectedPoint[0], debugImage.rows - 1 - projectedPoint[1]), 4, cv::Scalar(255, 0, 0), -1);
			   }
			   
		   }*/
		   
		   
		   if (p3D.size() > 5){
			   Math::Matrix<double, 6, 6> covar = Algorithm::PoseEstimation2D3D::singleCameraPoseError(headPose, p3D, intrinsics, 0.04f * 0.4f);


			   Math::Vector3d headPos = headPose.translation();

			   double quality = faceData.trackingQuality;

			   

			   double scaleFactor[3];


			   
			  

			   for (int j = 0; j < 3; j++){
				   covar(j, j) = covar(j, j)  * m_covarScalePos[j];
			   }

		
			   covar(3, 3) = covar(3, 3) * m_covarScaleRot[0];
			   covar(4, 4) = covar(4, 4) * m_covarScaleRot[1];
			   covar(5, 5) = covar(5, 5) * m_covarScaleRot[2];
			   m_outPortError.send(Measurement::ErrorPose(image.time(), Math::ErrorPose(headPose, covar)));
		   }
	  
      }
     
     
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
	cf->registerComponent< Ubitrack::Drivers::VisageFaceIPDEstimation >("VisageFaceIPDEstimation");

	
}

