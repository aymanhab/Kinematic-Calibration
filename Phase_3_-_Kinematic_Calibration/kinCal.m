function kinCal(varargin)
%kinCal
%
%   kinCal() main function for performing kinematic calibration in OpenSim.
%   
%   The input arguments to this function are:
%   - display: LSQNONLIN display option ('off', 'iter-detailed (default
%     setting), or 'final')
%   - maxFunEvals: maximum function evaluations (default is set to 200000)
%   - diffMinChange: minimum change in design variables for
%     finite-difference gradients (default is set to 10^-2)
%   
%   The function returns the optimized .osim model.
%   
%   Example function calls:
%   kinCal()
%   kinCal('off')
%   kinCal([],100000)
%   kinCal('final',[],10^-3)

%--------------------------------------------------------------------------
%setup
%--------------------------------------------------------------------------
fprintf(1,'\nSetting up kinematic calibration.\n')

%import OpenSim Matlab API functions
import org.opensim.modeling.*

%import set of utility functions created for the kinematic calibration
%routine
import utilFunctions.*

%check input arguments and populate variables accordingly
Nin=nargin;%number of input arguments
if Nin==0
    display='iter-detailed';%default display option
    maxFunEvals=200000;%default max function evaluations
    diffMinChange=10^-2;%default min change in design variables
elseif Nin==1
    display=varargin{1};%display option
    maxFunEvals=200000;%default max function evaluations
    diffMinChange=10^-2;%default min change in design variables
elseif Nin==2
    display=varargin{1};%display option
    maxFunEvals=varargin{2};%max function evaluations
    diffMinChange=10^-2;%default min change in design variables
elseif Nin==3
    display=varargin{1};%display option
    maxFunEvals=varargin{2};%max function evaluations
    diffMinChange=varargin{3};%min change in design variables
end

%locate input file containing settings for kinematic calibration
setupFile=dir('inputSetupFile.xml');

%read input setup file
[jointStruct,plateStruct,optSettings]=...
    utilFunctions.readInputSetupFile(setupFile.name);

%extract information from optSettings
inModel=optSettings.inModel;%input .osim model filename
outModel=optSettings.outModel;%output (optimized) model filename
accuracy=optSettings.accuracy;%inverse kinematics solver accuracy

%assign settings for Matlab's LSQNONLIN optimizer
options=optimset('lsqnonlin');%use LSQNONLIN
options=optimset(options,'Display',display);%display option
options=optimset(options,...
    'MaxFunEvals',maxFunEvals);%max function evaluations
options=optimset(options,'Algorithm','levenberg-marquardt');%algorithm
options=optimset(options,...
    'DiffMinChange',diffMinChange);%min change in variables
lb=[];%lower bounds
ub=[];%upper bounds

%setup OpenSim model
model=Model(inModel);
numOfModelJoints=model.getNumJoints;%get number of joints in the model
for ii=1:numOfModelJoints
    allJointNames{ii}=char(model.getJointSet...
        .get(ii-1).getName);%joint names
end

%assign values to params structure array for use in cost function
params.model=model;%OpenSim model
params.jointStruct=jointStruct;%joint structure array
params.plateStruct=plateStruct;%marker plate structure array
%--------------------------------------------------------------------------
%setup
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%perform kinematic calibration
%--------------------------------------------------------------------------
%determine number of optimizations that will be peformed
numOpts=numel(optSettings.OptimizationGroup);

%determine optimization order
for ii=1:numOpts
    optOrder=optSettings.OptimizationGroup(ii).optimizationOrder;
    optGroupings(optOrder)=optSettings.OptimizationGroup(ii);
end

%run optimazation(s) in order specified
for ii=1:numOpts
    %assign optimization group variable for ease of referencing
    optGroup=optGroupings(ii);
    
    %set which joints and marker plates are part of first optimization
    whichJoints=optGroup.whichJoints;%which joints
    whichMarkerPlates=optGroup.whichMarkerPlates;%which marker plates
    numOfJoints=numel(whichJoints);%num of defined joints
    numOfMarkerPlates=...
        numel(whichMarkerPlates);%num of defined marker plates
    
    %print which joints/marker plates are part of the optimization
    msg1='\nSetting up kinematic calibration for optimization group ';
    msg2=' , which includes joints/plates:\n       ';
    fprintf(1,[msg1 num2str(ii) msg2])
    
    %joints
    for jj=1:numOfJoints
        fprintf([whichJoints{jj} '\n       '])
    end
    
    %marker plates
    for jj=1:numOfMarkerPlates
        fprintf([whichMarkerPlates{jj} '\n       '])
    end
    
    %load optimization specific joint settings
    markerFileName=optGroup.expData;%experimental data filename
    ikTasksSetupFile=...
        optGroup.ikSetupFile;%inverse kinematics tasks setup filename
    startTime=optGroup.time_range(1);%simulation start time (sec)
    finalTime=optGroup.time_range(2);%simulation end time (sec)
    
    %setup inverse kinematics solver object for this optimization
    %ikTool object
    ikTool=InverseKinematicsTool(ikTasksSetupFile);
    %set constraintWeight
    constraintWeight=Inf;
    %markersReference object
    markersReference=MarkersReference();
    %markerWeights object
    markerWeights=SetMarkerWeights;
    %coordinateReferences object
    coordinateReferences=ArrayCoordinateReference;
    %ikTasks object
    ikTasks=ikTool.getIKTaskSet;
    %get weights and populate markerWeights and coordinateReferences
    %objects
    for jj=0:ikTasks.getSize-1
        objectName=ikTasks.get(jj);%get name of object
        if strcmp(objectName.getConcreteClassName,'IKMarkerTask')
            markerWeights.cloneAndAppend(MarkerWeight(objectName...
                .getName,objectName.getWeight));
        else
            value=model.getCoordinateSet().get(objectName.getName())...
                .getDefaultValue();
            reference=Constant(value);
            coordRef=CoordinateReference(objectName.getName(),reference);
            coordRef.setWeight(objectName.getWeight());
            coordinateReferences.push_back(coordRef);
        end
    end
    %set the weights for markers
    markersReference.setMarkerWeightSet(markerWeights);
    %load markers
    markersReference.loadMarkersFile(markerFileName);
    %ikSolver object
    ikSolver=InverseKinematicsSolver(model,markersReference,...
        coordinateReferences,constraintWeight);
    %set accuracy
    ikSolver.setAccuracy(accuracy);
    %sampling time (s)
    dt=1.0/markersReference.getSamplingFrequency();
    %number of frames
    nFrames=((finalTime-startTime)/dt)+1;
    %number of markers
    nMarkers=markerWeights.getSize();
    
    %save inverse kinematics related parameters in params structure array
    %for use in cost function
    params.ikTasks=ikTasks;%ikTasks object
    params.ikSolver=ikSolver;%inverse kinematics solver object
    params.startTime=startTime;%simulation start time (s)
    params.nFrames=nFrames;%number of marker frames
    params.nMarkers=nMarkers;%number of markers
    params.dt=dt;%sampling time (s)
    
    %calculate marker errors before optimization to compare with post
    %optimization errors
    [initMarkerErrs,initMarkerPos,markerNames]=utilFunctions...
        .calcMarkerErrors(model,ikSolver,dt,nFrames,nMarkers,startTime);
    
    %calculate root-mean-square (RMS) error for all time points for each
    %marker
    initMarkerRMS=sqrt(sum(initMarkerErrs.^2)/nFrames);
    
    %report errors to user
    fprintf('\nInitial marker RMS errors:\n')
    [markerNames;num2cell(initMarkerRMS)]
    
    %read marker data
    [expMarkerNames,expMarkerLocations]=utilFunctions...
        .readTrcFile(markerFileName);
    
    %remove unused marker data
    whichExpMarker=zeros(1,nMarkers);%intialize variable
    for jj=0:nMarkers-1
        [junk,whichExpMarker(jj+1)]=max(strcmp(expMarkerNames,...
            markerNames{jj+1}));
    end
    whichExpMarker=(whichExpMarker-1)*3+1;
    whichExpMarker3D=[whichExpMarker;whichExpMarker+1;whichExpMarker+2];
    %data that will be used in optimization
    expMarkerLocationsUsed=expMarkerLocations(:,whichExpMarker3D(:));
    
   %add used marker data to params structure array for use in cost
    %function
    params.expMarkerLocations=expMarkerLocationsUsed;
    
    %initialize optimization specific structure arrays
    [jointStructOpt,plateStructOpt]=utilFunctions...
        .initOptStruct(jointStruct,plateStruct,numOfJoints,...
        numOfMarkerPlates,whichJoints,whichMarkerPlates);
    
    %determine initial model parameter values for optimized parameters
    [vJoint,vJointMax]=utilFunctions.getParams(jointStructOpt,[],model);
    [vMarkerPlate,vMarkerPlateMax]=utilFunctions...
        .getParams([],plateStructOpt,model);
    
    %determine number of optimized parameters for each joint/plate
    numOfJointParams=zeros(1,numOfJoints);%num of joint parameters
    numOfMarkerPlateParams=...
        zeros(1,numOfMarkerPlates);%num of marker plate parameters
    for jj=1:numOfJoints
        numOfJointParams(jj)=sum([jointStructOpt.LocationInParent(jj,:)...
            jointStructOpt.OrientationInParent(jj,:)...
            jointStructOpt.LocationInChild(jj,:)...
            jointStructOpt.OrientationInChild(jj,:)]);
    end
    for jj=1:numOfMarkerPlates
        numOfMarkerPlateParams(jj)=sum...
            ([plateStructOpt.LocationInParent(jj,:)...
            plateStructOpt.OrientationInParent(jj,:)...
            plateStructOpt.LocationInChild(jj,:)...
            plateStructOpt.OrientationInChild(jj,:)]);
    end
    
    %build symmetry constraints
    if isempty(optGroup.SymmetricJointPair)
        %set no constraints flag
        params.isConstrained=0;
        
        %initialize dv parameters
        dvJoint0=zeros(size(vJoint));
        dvMarkerPlate0=zeros(size(vMarkerPlate));
    else
        cnstrPairIndices=utilFunctions...
            .buildSymConstraints(optGroup,jointStructOpt,numOfJointParams);
        
        %make matrix indicating constrained state of dv values
        dvJointUncon=ones(size(vJoint));
        dvJointUncon(abs(cnstrPairIndices(:,2)))=0;
        
        %add to params structure array for use in cost function
        params.dvJointUncon=dvJointUncon;
        params.cnstrPairIndices=cnstrPairIndices;
        params.isConstrained=1;
        
        %initialize dv parameters
        dvJoint0=zeros(size(vJoint));
        dvMarkerPlate0=zeros(size(vMarkerPlate));
        
        %remove duplicate parameters from dv for optimization speedup
        dvJoint0(abs(cnstrPairIndices(:,2)))=[];
    end %end build symmetry constraints if-statement
    
    %assign values to params structure array for use in cost function
    params.jointStruct=...
        jointStructOpt;%joint struct to be used inside cost function
    params.plateStruct=...
        plateStructOpt;%marker plate struct to be used inside cost function
    params.v=[vJoint;vMarkerPlate];%actual design parameters
    params.vMax=...
        [vJointMax;...
        vMarkerPlateMax];%max position change if penalization occurs
    params.numJointParamsV=...
        numel(vJoint);%number of joint parameters in dv after constraint 
                      %values are removed
    params.numMarkerPlateParamsV=...
        numel(vMarkerPlate);%number of marker plate parameters
    params.numJointParamsDV=...
        numel(dvJoint0);%number of joint parameters in dv after constraint 
                        %values are removed
    params.numMarkerPlateParamsDV=...
        numel(dvMarkerPlate0);%number of marker plate parameters
    params.penalizeChanges=...
        char(optGroup.penalizeChangesFlag);%flag to indicate if changes in 
                                           %dv parameters will be penalized
    
    %run optimization by calling LSQNONLIN
    msg='\nCalling LSQNONLIN for optimization group ';
    fprintf(1,[msg num2str(ii) '\n']);
    x=lsqnonlin(@costFunction,[dvJoint0;dvMarkerPlate0],...
        lb,ub,options,params);
    
    %process optimization output
    vMax=[vJointMax;vMarkerPlateMax];
    
    if (params.isConstrained)
        %read in parameters associated with constraints
        dvJointUncon=params.dvJointUncon;
        cnstrPairIndices=params.cnstrPairIndices;
        numJointParamsDV=params.numJointParamsDV;
        numJointParamsV=params.numJointParamsV;
        %divide parameters into joint and marker
        xJoint=x(1:numJointParamsDV);
        dvMarkerPlates=x(numJointParamsDV+1:end);
        %expand dvJoint to its original size
        dvJoint=zeros(numJointParamsV,1);
        %assign unconstrained values to dv
        dvJoint(logical(dvJointUncon))=xJoint;
        %assign constrained values to dv from their paired value
        dvJoint(abs(cnstrPairIndices(:,2)))=...
            sign(cnstrPairIndices(:,2)).*dvJoint(abs(cnstrPairIndices(:,1)));
        %unscale the design variable
        dv=[dvJoint; dvMarkerPlates].*vMax;
    else
        %unscale design variable, i.e., delta change
        dv=x.*vMax;
    end %end params.isConstrained if-statement
    
    %design parameters
    v=[vJoint;vMarkerPlate];
    
    %set model parameters
    utilFunctions.setParams(jointStructOpt,plateStructOpt,model,v,dv);
    
    %scale the model according to changes in joint parameters
    if char(optGroup.scaleSegments)
        %put all input arguments to rescaleModel in a structure array
        inputArg.v=v;
        inputArg.dv=dv;
        inputArg.vJoint=vJoint;
        inputArg.model=model;
        inputArg.numOfJoints=numOfJoints;
        inputArg.allJointNames=allJointNames;
        inputArg.jointStructOpt=jointStructOpt;
        inputArg.optGroup=optGroup;
        inputArg.numOfJointParams=numOfJointParams;
        inputArg.optSettings=optSettings;
        inputArg.markerPlateStructOpt=plateStructOpt;
        inputArg.numOfModelJoints=numOfModelJoints;
        %call rescaleModel
        utilFunctions.rescaleModel(inputArg);
    end
    
    %calculate new marker errors
    [newMarkerErrs,newMarkerPos,markerNames]=utilFunctions...
        .calcMarkerErrors(model,ikSolver,dt,nFrames,nMarkers,startTime);
    
    %calculate RMS error for all time points for each marker
    newMarkerRMS=sqrt(sum(newMarkerErrs.^2)/nFrames);
    
    %report errors to user
    fprintf('\nNew RMS marker errors:\n')
    [markerNames;num2cell(newMarkerRMS)]
    
end %end ii for-loop
%--------------------------------------------------------------------------
%perform kinematic calibration
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%create output (optimized) .osim model
%--------------------------------------------------------------------------
%output .osim model
model.setName(strrep(outModel,'.osim',''));
model.print(outModel);
fprintf('\nDone!\n')
%--------------------------------------------------------------------------
%create output (optimized) .osim model
%--------------------------------------------------------------------------