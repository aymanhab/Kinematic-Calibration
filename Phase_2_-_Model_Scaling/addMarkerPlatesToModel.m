function addMarkerPlatesToModel(varargin)
%addMarkerPlatesToModel
%
%   addMarkerPlatesToModel() adds marker plates to an existing .osim model.
%   
%   The input arguments to this function are the name of the .osim model
%   that plates are to be added and an output filename (optional). If an
%   output filename is not specified, a default name (platedModel.osim) is
%   used.
%   
%   The function returns the plated .osim model.
%   
%   Example function calls:
%   addMarkerPlatesToModel('inModel.osim')
%   addMarkerPlatesToModel('inModel.osim','outModel.osim')


%import OpenSim Matlab API functions
import org.opensim.modeling.*
import java.lang.*

%check input arguments and populate variables accordingly
Nin=nargin;%number of input arguments
if Nin==1
    inModel=varargin{1};%input .osim filename
    outModel='platedModel.osim';%default output filename
else
    inModel=varargin{1};%input .osim filename
    outModel=varargin{2};%output filename
end

%setup OpenSim model
model=Model(inModel);
%get number of model markers
numMarkers=model.getNumMarkers;
%initialize variables
markerBodyNames=cell(numMarkers,1);
markerOffsets=zeros(numMarkers,3);
%get marker body names and positions in each body
for ii=1:numMarkers
    marker=model.getMarkerSet.get(ii-1);
    markerBody=marker.getBodyName;
    markerBodyNames{ii}=char(markerBody);    
    markerOffset=char(marker.getOffset);
    markerOffset=str2num(markerOffset(2:end));
    markerOffsets(ii,:)=markerOffset;
end
%eliminate repetitive marker body names
bodies=unique(markerBodyNames);
plateName=bodies;%plate names
jointName=bodies;%joint names
%initialize plateLocationInParent
plateLocationInParent=zeros(numel(bodies),3);
newMarkerOffsets=markerOffsets;%new marker offsets
%initialize additional variables
vec3Zeros=Vec3(0);
inertiaVec=ArrayDouble(0,6);
%find origin locations for marker plate bodies
for ii=1:numel(bodies)
    markersInBody=strcmp(markerBodyNames,bodies{ii});
    plateLocationInParent(ii,:)=mean(markerOffsets(markersInBody,:));
    newMarkerOffset(markersInBody,:)=...
        markerOffsets(markersInBody,:)-...
        ones(sum(markersInBody),1)*plateLocationInParent(ii,:);
    plateName{ii}=[bodies{ii} '_markerPlate'];
    jointName{ii}=[plateName{ii} '_jnt'];
    body=Body();
    body.setMass(0);
    body.setMassCenter(vec3Zeros);
    body.setInertia(inertiaVec);
    msg1='\n\nCreating marker plate named: ';
    msg2=' with weld joint ';
    msg3=' with parent body ';
    msg4='.\n\n';
    fprintf(1,...
        [msg1 plateName{ii} msg2 jointName{ii} msg3 bodies{ii} msg4]);
    body.setName(plateName{ii});
    plateLocationInParentVec3=...
        Vec3(plateLocationInParent(ii,1),plateLocationInParent(ii,2),...
        plateLocationInParent(ii,3));
    parentBody=model.getBodySet.get(bodies{ii});
    joint=WeldJoint(String(jointName{ii}),parentBody,...
        plateLocationInParentVec3,vec3Zeros,body,vec3Zeros,vec3Zeros,0);
    body.setJoint(joint);
    model.addBody(body);
end %end ii for-loop
%create new marker offsets
for ii=1:numMarkers
   marker=model.getMarkerSet.get(ii-1);
   markerBody=char(marker.getBodyName);
   marker.setBodyName(String([markerBody '_markerPlate']));
   marker.setOffset(Vec3(newMarkerOffset(ii,1),newMarkerOffset(ii,2),...
       newMarkerOffset(ii,3)));
end
%output, i.e., plate, .osim model
model.setName(outModel);%set name
model.print(outModel);%print