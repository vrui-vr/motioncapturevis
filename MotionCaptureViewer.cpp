/***********************************************************************
MotionCaptureViewer - Viewer for motion capture traces collected as time
series of raw marker positions.
Copyright (c) 2026 Oliver Kreylos
***********************************************************************/

#include <stddef.h>
#include <string>
#include <vector>
#include <Misc/StdError.h>
#include <Misc/MessageLogger.h>
#include <Misc/RGB.h>
#include <Misc/ColorMap.h>
#include <IO/OpenFile.h>
#include <IO/CSVSource.h>
#include <Math/Interval.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/LinearUnit.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <SceneGraph/GroupNode.h>
#include <SceneGraph/ONTransformNode.h>
#include <SceneGraph/AppearanceNode.h>
#include <SceneGraph/SphereNode.h>
#include <SceneGraph/ColorNode.h>
#include <SceneGraph/CoordinateNode.h>
#include <SceneGraph/IndexedLineSetNode.h>
#include <SceneGraph/ShapeNode.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Vrui/CoordinateManager.h>
#include <Vrui/SceneGraphManager.h>

// DEBUGGING
#include <iostream>

class MotionCaptureViewer:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef float Scalar;
	typedef Geometry::Point<Scalar,3> Point;
	
	struct Marker // Structure containing a potentially invalid marker position
		{
		/* Elements: */
		public:
		bool valid; // Flag if the marker position is valid in the current frame
		Point position; // Marker position
		
		/* Constructors and destructors: */
		public:
		Marker(bool sValid,const Point& sPosition) // Elementwise constructor
			:valid(sValid),position(sPosition)
			{
			}
		};
	
	struct MarkerTrace // Structure containing a sequence of time-stamped marker positions
		{
		/* Elements: */
		public:
		std::string name; // Name assigned to the marker
		std::string id; // System-assigned unique ID of the marker
		std::vector<Marker> trace; // The marker's trace
		
		/* Constructors and destructors: */
		MarkerTrace(const std::string& sName,const std::string& sId) // Creates an empty marker trace for the given marker name and ID
			:name(sName),id(sId)
			{
			}
		};
	
	/* Elements: */
	std::string takeName; // The name of the motion capture take
	double frameRate; // Frame rate of motion capture data exported to the file (not really necessary due to explicit time stamps)
	size_t totalNumFrames; // Total number of frames in the take
	Geometry::LinearUnit positionUnit; // Unit for marker positions
	std::vector<double> frameTimes; // List of time stamps associated with marker trace frames
	std::vector<MarkerTrace> markerTraces; // List of time sequences of marker positions
	Math::Interval<double> timeRange; // The total range of valid marker positions in any of the traces
	SceneGraph::ONTransformNodePointer root; // Root of the static and individual marker visualization
	SceneGraph::ShapeNodePointer staticVisualization; // Static visualization of the motion capture take
	SceneGraph::GroupNodePointer markerGroup; // Group node containing the individual marker visualizations
	SceneGraph::ShapeNodePointer* markers; // Array of visualizations for the motion capture take's individual markers
	bool* markerValids; // Array of flags if the motion capture take's individual markers are currently visible
	GLMotif::PopupWindow* playbackDialogPopup; // The playback control dialog
	GLMotif::TextFieldSlider* timeSlider; // Slider to select the current playback time
	GLMotif::ToggleButton* playToggle; // Toggle button to enable/disable automatic playback
	bool playback; // Flag whether automatic playback is enabled
	bool playbackLooping; // Flag whether playback restarts automatically when it reaches the end of the motion capture trace
	double playbackTime; // The current playback time
	double playbackFactor; // The playback speed-up factor
	
	/* Private methods: */
	void readTraceFile(const char* fileName); // Reads a trace file in CSV format and creates the list of marker traces
	void updateMarkerVisualization(double visualizationTime); // Updates the marker visualization to the given time point
	void timeSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void playToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	
	/* Constructors and destructors: */
	public:
	MotionCaptureViewer(int& argc,char**& argv);
	virtual ~MotionCaptureViewer(void);
	
	/* Methods from class Vrui::Application: */
	virtual void frame(void);
	virtual void resetNavigation(void);
	};

/************************************
Methods of class MotionCaptureViewer:
************************************/

void MotionCaptureViewer::readTraceFile(const char* fileName)
	{
	/* Open the file of the given name as a CSV file: */
	IO::CSVSource csv(IO::openFile(fileName));
	
	/*********************************************************************
	Retrieve format information and metadata:
	*********************************************************************/
	
	/* Check the format tag: */
	if(csv.readField<std::string>()!="Format Version")
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"%s is not an OptiTrack csv trace file",fileName);
	std::string formatVersion=csv.readField<std::string>();
	if(formatVersion!="1.25")
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"%s has unsupported format version %s",fileName,formatVersion.c_str());
	
	/* Read the take name: */
	csv.skipField();
	takeName=csv.readField<std::string>();
	
	/* Read the export frame rate: */
	for(int i=0;i<5;++i)
		csv.skipField();
	frameRate=csv.readField<double>();
	
	/* Read the total number of exported frames: */
	for(int i=0;i<7;++i)
		csv.skipField();
	totalNumFrames=csv.readField<unsigned int>();
	
	/* Read the position unit: */
	for(int i=0;i<3;++i)
		csv.skipField();
	std::string unitName=csv.readField<std::string>();
	if(unitName=="Millimeters")
		positionUnit=Geometry::LinearUnit(Geometry::LinearUnit::MILLIMETER,1.0);
	else if(unitName=="Meters")
		positionUnit=Geometry::LinearUnit(Geometry::LinearUnit::METER,1.0);
	else
		{
		/* Print an error message and carry on: */
		Misc::sourcedUserWarning(__PRETTY_FUNCTION__,"%s has unrecognized position unit %s",fileName,unitName.c_str());
		}
	
	/* Skip the rest of the first record and the empty second record: */
	csv.skipRecord();
	csv.skipRecord();
	
	/*********************************************************************
	Retrieve the marker trace layout:
	*********************************************************************/
	
	/* Determine the number of markers: */
	for(int i=0;i<2;++i)
		csv.skipField();
	
	size_t numMarkers=0;
	while(!csv.eor())
		{
		/* Check if the next triplet of columns defines a marker: */
		int numColumns=0;
		for(;numColumns<3&&!csv.eor()&&csv.readField<std::string>()=="Marker";++numColumns)
			;
		if(numColumns<3)
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"%s contains non-marker traces",fileName);
		
		++numMarkers;
		}
	
	/* Read the list of marker names: */
	for(int i=0;i<2;++i)
		csv.skipField();
	std::vector<std::string> markerNames;
	markerNames.reserve(numMarkers);
	for(size_t i=0;i<numMarkers;++i)
		{
		/* Read the marker name: */
		if(csv.eor())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker names in %s",fileName);
		markerNames.push_back(csv.readField<std::string>());
		
		/* Ensure that the next two columns' names are identical to the marker name: */
		if(csv.eor()||csv.readField<std::string>()!=markerNames.back()||csv.eor()||csv.readField<std::string>()!=markerNames.back())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker names in %s",fileName);
		}
	if(!csv.eor())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker names in %s",fileName);
	
	/* Read the list of marker IDs: */
	for(int i=0;i<2;++i)
		csv.skipField();
	std::vector<std::string> markerIds;
	markerIds.reserve(numMarkers);
	for(size_t i=0;i<numMarkers;++i)
		{
		/* Read the marker ID: */
		if(csv.eor())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker IDs in %s",fileName);
		markerIds.push_back(csv.readField<std::string>());
		
		/* Ensure that the next two columns' IDs are identical to the marker ID: */
		if(csv.eor()||csv.readField<std::string>()!=markerIds.back()||csv.eor()||csv.readField<std::string>()!=markerIds.back())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker IDs in %s",fileName);
		}
	if(!csv.eor())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker IDs in %s",fileName);
	
	/* Skip the next record containing marker parents: */
	csv.skipRecord();
	
	/* Ensure that all columns contain marker positions: */
	for(int i=0;i<2;++i)
		csv.skipField();
	for(size_t i=0;i<numMarkers;++i)
		for(int j=0;j<3;++j)
			if(csv.eor()||csv.readField<std::string>()!="Position")
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker definitions in %s",fileName);
	if(!csv.eor())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker definitions in %s",fileName);
	
	/* Ensure that all columns contain marker positions: */
	for(int i=0;i<2;++i)
		csv.skipField();
	for(size_t i=0;i<numMarkers;++i)
		for(int j=0;j<3;++j)
			{
			char fieldName[2];
			fieldName[0]='X'+j;
			fieldName[1]='\0';
			if(csv.eor()||csv.readField<std::string>()!=fieldName)
				throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker definitions in %s",fileName);
			}
	if(!csv.eor())
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker definitions in %s",fileName);
	
	/* Create the list of marker traces: */
	markerTraces.clear();
	markerTraces.reserve(numMarkers);
	for(size_t i=0;i<numMarkers;++i)
		markerTraces.push_back(MarkerTrace(markerNames[i],markerIds[i]));
	
	/*********************************************************************
	Read all marker traces:
	*********************************************************************/
	
	while(!csv.eof())
		{
		/* Read the trace frame time: */
		unsigned int frameIndex=csv.readField<unsigned int>();
		if(csv.eor())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker trace frame %u in %s",frameIndex,fileName);
		double frameTime=csv.readField<double>();
		frameTimes.push_back(frameTime);
		
		/* Read all marker positions that were valid during this frame: */
		for(size_t i=0;i<numMarkers;++i)
			{
			Point pos;
			bool posValid=true;
			for(int j=0;j<3;++j)
				{
				if(csv.eor())
					throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker trace frame %u in %s",frameIndex,fileName);
				
				/* Read and parse the position value and check for invalid or empty values: */
				const char* posString=csv.readFieldIntoBuffer();
				char* posEnd;
				pos[j]=Scalar(strtod(posString,&posEnd));
				if(*posEnd!='\0')
					throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Number format error for marker %u in marker trace frame %u in %s",(unsigned int)i,frameIndex,fileName);
				posValid=posValid&&posEnd!=posString;
				}
			
			markerTraces[i].trace.push_back(Marker(posValid,pos));
			}
		if(!csv.eor())
			throw Misc::makeStdErr(__PRETTY_FUNCTION__,"Format error in marker trace frame %u in %s",frameIndex,fileName);
		}
	
	/* Calculate the motion capture data's effective time range: */
	timeRange=Math::Interval<double>::empty;
	for(std::vector<MarkerTrace>::iterator mtIt=markerTraces.begin();mtIt!=markerTraces.end();++mtIt)
		for(size_t frameIndex=0;frameIndex<frameTimes.size();++frameIndex)
			if(mtIt->trace[frameIndex].valid)
				timeRange.addValue(frameTimes[frameIndex]);
	
	/* Create a root transformation node to make the entire visualization track the current frame's centroid: */
	root=new SceneGraph::ONTransformNode;
	
	/* Create a static visualization of the trace file: */
	Misc::ColorMap<Misc::RGB<float> > traceColorMap;
	traceColorMap.insertEntry(0,timeRange.fractionToValue(0.0),Misc::RGB<float>(1.0f,0.0f,0.0f));
	traceColorMap.insertEntry(1,timeRange.fractionToValue(0.2),Misc::RGB<float>(1.0f,1.0f,0.0f));
	traceColorMap.insertEntry(2,timeRange.fractionToValue(0.4),Misc::RGB<float>(0.0f,1.0f,0.0f));
	traceColorMap.insertEntry(3,timeRange.fractionToValue(0.6),Misc::RGB<float>(0.0f,1.0f,1.0f));
	traceColorMap.insertEntry(4,timeRange.fractionToValue(0.8),Misc::RGB<float>(0.0f,0.0f,1.0f));
	traceColorMap.insertEntry(5,timeRange.fractionToValue(1.0),Misc::RGB<float>(1.0f,0.0f,1.0f));
	
	SceneGraph::ColorNodePointer color=new SceneGraph::ColorNode;
	SceneGraph::MFColor::ValueList& colors=color->color.getValues();
	
	SceneGraph::CoordinateNodePointer coord=new SceneGraph::CoordinateNode;
	SceneGraph::MFPoint::ValueList& coords=coord->point.getValues();
	
	SceneGraph::IndexedLineSetNodePointer lineSet=new SceneGraph::IndexedLineSetNode;
	lineSet->color.setValue(color);
	lineSet->coord.setValue(coord);
	lineSet->colorPerVertex.setValue(true);
	SceneGraph::MFInt::ValueList& coordIndices=lineSet->coordIndex.getValues();
	
	int vertexIndex=0;
	for(std::vector<MarkerTrace>::iterator mtIt=markerTraces.begin();mtIt!=markerTraces.end();++mtIt)
		{
		bool lastValid=false;
		for(size_t frameIndex=0;frameIndex<frameTimes.size();++frameIndex)
			{
			bool valid=mtIt->trace[frameIndex].valid;
			if(valid)
				{
				colors.push_back(SceneGraph::Color(traceColorMap.map(frameTimes[frameIndex]).getComponents()));
				coords.push_back(mtIt->trace[frameIndex].position);
				coordIndices.push_back(vertexIndex);
				++vertexIndex;
				}
			else if(lastValid)
				coordIndices.push_back(-1);
			lastValid=valid;
			}
		if(lastValid)
			coordIndices.push_back(-1);
		}
	color->update();
	coord->update();
	lineSet->update();
	
	staticVisualization=new SceneGraph::ShapeNode;
	staticVisualization->geometry.setValue(lineSet);
	staticVisualization->update();
	root->addChild(*staticVisualization);
	
	/* Create the individual marker visualizations: */
	markerGroup=new SceneGraph::GroupNode;
	root->addChild(*markerGroup);
	
	markers=new SceneGraph::ShapeNodePointer[numMarkers];
	markerValids=new bool[numMarkers];
	for(size_t i=0;i<numMarkers;++i)
		{
		SceneGraph::AppearanceNodePointer appearance=SceneGraph::createDiffuseAppearance(SceneGraph::Color(0.667f,0.667f,0.667f));
		
		SceneGraph::SphereNodePointer sphere=new SceneGraph::SphereNode;
		sphere->radius.setValue(SceneGraph::Scalar(positionUnit.getMeterFactor()*0.01));
		sphere->useImpostor.setValue(true);
		sphere->ccw.setValue(true);
		sphere->update();
		
		markers[i]=new SceneGraph::ShapeNode;
		markers[i]->appearance.setValue(appearance);
		markers[i]->geometry.setValue(sphere);
		markers[i]->update();
		
		markerValids[i]=false;
		}
	}

void MotionCaptureViewer::updateMarkerVisualization(double visualizationTime)
	{
	/* Find the trace frame corresponding to the given visualization time: */
	size_t left=0;
	size_t right=frameTimes.size()+1;
	while(left<right-1)
		{
		size_t mid=(left+right)/2;
		if(frameTimes[mid-1]<=visualizationTime)
			left=mid;
		else
			right=mid;
		}
	if(left==frameTimes.size()&&visualizationTime==frameTimes[left-1])
		--left;
	if(left>=1&&left<frameTimes.size())
		{
		/* Update the marker visualizations for the found frame: */
		size_t frameIndex=left-1;
		Scalar interpolator((visualizationTime-frameTimes[frameIndex])/(frameTimes[frameIndex+1]-frameTimes[frameIndex]));
		Geometry::AffineCombiner<Scalar,3> centroidC;
		for(size_t i=0;i<markerTraces.size();++i)
			{
			/* Update the marker's visualization if it is valid in the found frame: */
			std::vector<Marker>& trace=markerTraces[i].trace;
			bool valid=trace[frameIndex].valid&&trace[frameIndex+1].valid;
			if(valid)
				{
				/* Interpolate the marker position based on the surrounding valid positions: */
				SceneGraph::SphereNodePointer sphere(markers[i]->geometry.getValue());
				Point markerPos=Geometry::affineCombination(trace[frameIndex].position,trace[frameIndex+1].position,interpolator);
				centroidC.addPoint(markerPos);
				sphere->center.setValue(SceneGraph::Point(markerPos));
				sphere->update();
				
				if(!markerValids[i])
					markerGroup->addChild(*markers[i]);
				}
			else
				{
				/* Find the two valid marker positions bracketing the current frame: */
				size_t lastValidPlus1;
				for(lastValidPlus1=frameIndex+1;lastValidPlus1>0&&!trace[lastValidPlus1-1].valid;--lastValidPlus1)
					;
				size_t nextValid;
				for(nextValid=frameIndex+1;nextValid<frameTimes.size()&&!trace[nextValid].valid;++nextValid)
					;
				if(lastValidPlus1>0&&nextValid<frameTimes.size())
					{
					/* Interpolate a marker position for the purpose of smooth centroid tracking: */
					Scalar invalidInterpolator((visualizationTime-frameTimes[lastValidPlus1-1])/(frameTimes[nextValid]-frameTimes[lastValidPlus1-1]));
					Point markerPos=Geometry::affineCombination(trace[lastValidPlus1-1].position,trace[nextValid].position,invalidInterpolator);
					centroidC.addPoint(markerPos);
					}
				else if(lastValidPlus1>0)
					{
					Point markerPos=trace[lastValidPlus1-1].position;
					centroidC.addPoint(markerPos);
					}
				else
					{
					Point markerPos=trace[nextValid].position;
					centroidC.addPoint(markerPos);
					}
				
				if(markerValids[i])
					markerGroup->removeChild(*markers[i]);
				}
			markerValids[i]=valid;
			}
		
		if(centroidC.isValid())
			{
			/* Move the entire visualization to the centroid of the currently valid marker set: */
			Point center=centroidC.getPoint();
			root->setTransform(SceneGraph::ONTransform::translateToOriginFrom(center));
			}
		}
	}

void MotionCaptureViewer::timeSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the current playback time: */
	playbackTime=cbData->value;
	
	/* If automatic playback is enabled, set the selected time back by a bit so that it will be shown exactly in the current frame: */
	if(playback)
		playbackTime-=Vrui::getFrameTime()*playbackFactor;
	else
		{
		/* Immediately update the marker visualization: */
		updateMarkerVisualization(playbackTime);
		}
	}

void MotionCaptureViewer::playToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	if(cbData->set)
		{
		/* If we're at the end of the trace, start again from the beginning: */
		if(playbackTime>=timeRange.getMax())
			playbackTime=timeRange.getMin();
		
		/* Set the current playback time back a bit so that it will be shown exactly in the current frame: */
		playbackTime-=Vrui::getFrameTime()*playbackFactor;
		
		playback=true;
		}
	else
		playback=false;
	}

MotionCaptureViewer::MotionCaptureViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 frameRate(0),totalNumFrames(0),
	 markers(0),markerValids(0),
	 playbackDialogPopup(0),
	 playback(false),playbackLooping(false),playbackTime(0),playbackFactor(1.0)
	{
	/* Read the motion capture trace file: */
	if(argc<2)
		throw Misc::makeStdErr(__PRETTY_FUNCTION__,"No motion capture trace file provided");
	readTraceFile(argv[1]);
	playbackTime=timeRange.getMin();
	
	/* Add the motion capture trace's static and individual marker visualizations to the navigational scene graph: */
	Vrui::getSceneGraphManager()->addNavigationalNode(*root);
	updateMarkerVisualization(playbackTime);
	
	/* Create a popup shell to hold the playback control dialog: */
	playbackDialogPopup=new GLMotif::PopupWindow("PlaybackDialogPopup",Vrui::getWidgetManager(),"Playback Control");
	playbackDialogPopup->setResizableFlags(true,false);
	
	/* Create a rowcolumn to hold the playback controls: */
	GLMotif::RowColumn* playbackDialog=new GLMotif::RowColumn("PlaybackDialog",playbackDialogPopup,false);
	playbackDialog->setOrientation(GLMotif::RowColumn::VERTICAL);
	playbackDialog->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	playbackDialog->setNumMinorWidgets(1);
	
	/* Create the time slider: */
	timeSlider=new GLMotif::TextFieldSlider("TimeSlider",playbackDialog,10,Vrui::getWidgetManager()->getStyleSheet()->fontHeight*20.0f);
	timeSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	timeSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	timeSlider->setValueRange(timeRange.getMin(),timeRange.getMax(),0.1);
	timeSlider->setValue(playbackTime);
	timeSlider->getValueChangedCallbacks().add(this,&MotionCaptureViewer::timeSliderCallback);
	
	/* Create the playback control area: */
	GLMotif::RowColumn* playbackControl=new GLMotif::RowColumn("PlaybackControl",playbackDialog,false);
	playbackControl->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	playbackControl->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	playbackControl->setNumMinorWidgets(1);
	
	/* Create the playback and loop toggles: */
	playToggle=new GLMotif::ToggleButton("PlayToggle",playbackControl,"Play");
	playToggle->setToggle(playback);
	playToggle->getValueChangedCallbacks().add(this,&MotionCaptureViewer::playToggleCallback);
	
	GLMotif::ToggleButton* loopToggle=new GLMotif::ToggleButton("LoopToggle",playbackControl,"Loop");
	loopToggle->track(playbackLooping);
	
	/* Create the speed-up slider: */
	GLMotif::TextFieldSlider* speedSlider=new GLMotif::TextFieldSlider("SpeedSlider",playbackControl,8,Vrui::getWidgetManager()->getStyleSheet()->fontHeight*10.0f);
	speedSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	speedSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	speedSlider->setValueRange(0.001,10.0,0.01);
	speedSlider->getSlider()->addNotch(0.0);
	speedSlider->track(playbackFactor);
	
	playbackControl->setColumnWeight(2,1.0f);
	playbackControl->manageChild();
	
	playbackDialog->manageChild();
	
	Vrui::popupPrimaryWidget(playbackDialogPopup);
	
	/* Set Vrui's navigational-space unit: */
	Vrui::getCoordinateManager()->setUnit(positionUnit);
	}

MotionCaptureViewer::~MotionCaptureViewer(void)
	{
	/* Delete the marker visualizations: */
	delete[] markers;
	delete[] markerValids;
	
	/* Delete the GUI: */
	delete playbackDialogPopup;
	}

void MotionCaptureViewer::frame(void)
	{
	if(playback)
		{
		/* Increment the current playback time: */
		playbackTime+=Vrui::getFrameTime()*playbackFactor;
		
		/* Check if playback is over: */
		if(playbackTime>timeRange.getMax())
			{
			if(playbackLooping)
				{
				/* Reset the playback time to the beginning of the motion capture trace: */
				playbackTime=timeRange.getMin();
				}
			else
				{
				/* Stop playback: */
				playbackTime=timeRange.getMax();
				playToggle->setToggle(false);
				playback=false;
				}
			}
		
		/* Update the marker visualization and the time slider: */
		updateMarkerVisualization(playbackTime);
		timeSlider->setValue(playbackTime);
		}
	
	/* Schedule another frame if playback is still enabled: */
	if(playback)
		Vrui::scheduleUpdate(Vrui::getNextAnimationTime());
	}

void MotionCaptureViewer::resetNavigation(void)
	{
	/* Calculate the bounding box of all marker positions: */
	Geometry::Box<Scalar,3> bbox=Geometry::Box<Scalar,3>::empty;
	for(std::vector<MarkerTrace>::iterator mtIt=markerTraces.begin();mtIt!=markerTraces.end();++mtIt)
		{
		for(std::vector<Marker>::iterator tIt=mtIt->trace.begin();tIt!=mtIt->trace.end();++tIt)
			bbox.addPoint(tIt->position);
		}
	
	/* Translate the bounding box by the current visualization transformation: */
	bbox.min=root->transform.getValue().transform(bbox.min);
	bbox.max=root->transform.getValue().transform(bbox.max);
	
	/* Center the display on the marker positions' bounding box: */
	Vrui::setNavigationTransformation(bbox,Vrui::Vector(0,0,1),Vrui::Vector(0,1,0));
	}

/****************
Main entry point:
****************/

VRUI_APPLICATION_RUN(MotionCaptureViewer)
