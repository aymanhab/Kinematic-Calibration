function varargout=costFunction(varargin)
%costFunction
%   
%   costFunction() sets up the cost function for the kinematic calibration
%   routine.
%   
%   The input arguments to this function are the design parameters and a
%   structure array containing optimization-related variables.

%import OpenSim Matlab API functions
import org.opensim.modeling.*

%import set of utility functions created for the kinematic calibration
%routine
import utilFunctions.*

%populate variables
x=varargin{1};%design parameters
params=varargin{2};%optimization-related variables

%unpack non-constraint-related variables from params
model=params.model;%OpenSim model
jointStruct=params.jointStruct;%joint structure array
plateStruct=params.plateStruct;%marker plate structure array
ikSolver=params.ikSolver;%inverse kinematics solver object
v=params.v;%actual model parameter, i.e., thigh length
vMax=params.vMax;%max position change
startTime=params.startTime;%simulation start time (s)
nFrames=round(params.nFrames);%number of marker frames
nMarkers=params.nMarkers;%number of markers
dt=params.dt;%sampling time (s)
penalizeChanges=params.penalizeChanges;
expMarkerLocations=params.expMarkerLocations;%experimental marker locations

%assign symmetry values, if applied
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
        sign(cnstrPairIndices(:,2)).*dvJoint(abs...
        (cnstrPairIndices(:,1)));
    
    %unscale the design variable
    dv=[dvJoint; dvMarkerPlates].*vMax;
else
    %unscale design variable, i.e., delta change
    dv=x.*vMax;
end

%set model params by calling local function setParams()
utilFunctions.setParams(jointStruct,plateStruct,model,v,dv);

state=model.initSystem;%initialize system

state.setTime(startTime);%set simulation start time
ikSolver.assemble(state);%assemble

%initialize variable
locations=zeros(nMarkers,3);
markerLocations=zeros(nFrames-1,3*nMarkers);

%run inverse kinematics
for ii=1:nFrames-1
    state.setTime(startTime+ii*dt);
    ikSolver.track(state);
    for jj=0:nMarkers-1
        %compute marker location
        currentLocation=ikSolver.computeCurrentMarkerLocation(jj);
        %get marker location from Vec3
        locations(jj+1,1)=currentLocation.get(0);
        locations(jj+1,2)=currentLocation.get(1);
        locations(jj+1,3)=currentLocation.get(2);
    end
    %store values
    markerLocations(ii,:)=reshape(locations',[1 numel(locations)]);
end
if size(expMarkerLocations,1)-size(markerLocations,1)==1
    markerError=expMarkerLocations(2:end,:)-markerLocations;
elseif size(expMarkerLocations,1)-size(markerLocations,1)==2
    markerError=expMarkerLocations(3:end,:)-markerLocations;
else
    markerError=expMarkerLocations-markerLocations;
end

%set output cost depending on whether changes in marker parameters should
%be penalized
if strcmp(penalizeChanges,'false')
    f=markerError(:);
else
    f=[markerError(:); x(:)*nFrames/100];
end

%prepare output
varargout={f};