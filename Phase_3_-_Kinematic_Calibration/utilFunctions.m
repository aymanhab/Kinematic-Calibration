%utilFunctions
%   utilFunctions contains all the utility functions used by 
%   the kinematic calibration routine.
%
%   List of utility functions
%   - readInputSetupFile()
%   - getParams()
%   - setParams()
%   - getCaseNumber()
%   - modifyVariable()
%   - calcMarkerErrors()
%   - initOptStruct()
%   - buildSymConstraints()
%   - readTrcFile()
%   - rescaleModel()

%start classdef
classdef utilFunctions
    %start methods
    methods (Static=true)
        %==================================================================
        %   readInputSetupFile
        %==================================================================
        function varargout=readInputSetupFile(varargin)
        %readInputSetupFile
        %
        %   readInputSetupFile() reads the XML input setup file (using 3rd
        %   party code, xmlRead, written by Jarek Tuszynski) to collect
        %   user-specified joint parameter, marker plate parameter, and
        %   input settings information.
        %   
        %   The input argument to the function is the name of the input
        %   setup file.
        %   
        %   The function returns three structure arrays containing
        %   information about joint parameters, marker plate parameters,
        %   and input settings, respectively. These arrays are used in the
        %   main kinematic calibration routine.
        %   
        %   Example function call:
        %   [jS,mS,iS]=readInputSetupFile('inputSetupFile.xml')
        
        %read setup file by calling local function xmlRead
        inputFile=xmlRead(varargin{1});
        
        %check JointParameters tag
        jointParamsChk=isfield(inputFile.startFile.objects,...
            'JointParameters');
        
        %check MarkerPlateParameters tag
        plateParamsChk=isfield(inputFile.startFile.objects,...
            'MarkerPlateParameters');
        
        %check InputSettings tag
        inputParamsChk=isfield(inputFile.startFile.objects,...
            'InputSettings');
        
        %if there is a JointParameters tag, then extract its content
        if (jointParamsChk)
            %get field(s)
            jointParams=getfield(inputFile.startFile.objects,...
                'JointParameters');
            %the structure may contain COMMENT and ATTRIBUTE fields; if
            %so, then remove these fields
            if (isfield(jointParams,'COMMENT'))
                jointParams=rmfield(jointParams,'COMMENT');
            end
            if (isfield(jointParams,'ATTRIBUTE'))
                jointParams=rmfield(jointParams,'ATTRIBUTE');
            end
            %see how many jointParams structure arrays are availabel
            [m,n]=size(jointParams);
            ln=max(m,n);
            %initialize following variables to avoid memory problems
            %also, by defining these parameters, if the user has not
            %specified a tag, the routine will know that the appropriate
            %tag parameters are not included in the optimization (i.e.,
            %they are set to zero)
            LocationInParent=zeros(ln,3);%location in parent (LiP)
            OrientationInParent=zeros(ln,3);%orientation in parent (OiP)
            LocationInChild=zeros(ln,3);%location in child (LiC)
            OrientationInChild=zeros(ln,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(ln,3);%LiP max change
            OrientationInParentMaxChange=zeros(ln,3);%OiP max change
            LocationInChildMaxChange=zeros(ln,3);%LiC max change
            OrientationInChildMaxChange=zeros(ln,3);%OiC max change
            %loop through and get jointParams content
            for ii=1:ln
                %joint flag
                if isfield(jointParams(ii),'apply')
                    if ~isempty(jointParams(ii).apply)
                        jointFlag(ii)={jointParams(ii).apply};
                    else
                        jointFlag(ii)={'notSpecified'};
                    end
                else
                    jointFlag={};
                end
                %joint name
                if isfield(jointParams(ii),'joint_name')
                    if ~isempty(jointParams(ii).joint_name)
                        jointName(ii)={jointParams(ii).joint_name};
                    else
                        jointName(ii)={'notSpecified'};
                    end
                    else
                    jointName={};
                end
                %joint type
                if isfield(jointParams(ii),'joint_type')
                    if ~isempty(jointParams(ii).joint_type)
                        jointType(ii)={jointParams(ii).joint_type};
                    else
                        jointType(ii)={'notSpecified'};
                    end
                else
                    jointType={};
                end
                %location_in_parent
                if isfield(jointParams(ii),'location_in_parent_params')
                    if ~isempty(jointParams(ii).location_in_parent_params)
                        LocationInParent(ii,:)=...
                            jointParams(ii).location_in_parent_params;
                    else
                        LocationInParent(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_parent
                if isfield(jointParams(ii),'orientation_in_parent_params')
                    if ~isempty(jointParams(ii)...
                            .orientation_in_parent_params)
                        OrientationInParent(ii,:)=...
                            jointParams(ii).orientation_in_parent_params;
                    else
                        OrientationInParent(ii,:)=zeros(1,3);
                    end
                end
                %location_in_child
                if isfield(jointParams(ii),'location_in_child_params')
                    if ~isempty(jointParams(ii).location_in_child_params)
                        LocationInChild(ii,:)=...
                            jointParams(ii).location_in_child_params;
                    else
                        LocationInChild(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_child
                if isfield(jointParams(ii),'orientation_in_child_params')
                    if ~isempty(jointParams(ii)...
                            .orientation_in_child_params)
                        OrientationInChild(ii,:)=...
                            jointParams(ii).orientation_in_child_params;
                    else
                        OrientationInChild(ii,:)=zeros(1,3);
                    end
                end
                %location_in_parent maximum allowable change
                if isfield(jointParams(ii),...
                        'location_in_parent_maxParamChange')
                    if ~isempty(jointParams(ii)...
                            .location_in_parent_maxParamChange)
                        LocationInParentMaxChange(ii,:)=jointParams(ii)...
                            .location_in_parent_maxParamChange;
                    else
                        LocationInParentMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_parent maximum allowable change
                if isfield(jointParams(ii),...
                        'orientation_in_parent_maxParamChange')
                    if ~isempty(jointParams(ii)...
                            .orientation_in_parent_maxParamChange)
                        OrientationInParentMaxChange(ii,:)=...
                            jointParams(ii)...
                            .orientation_in_parent_maxParamChange;
                    else
                        OrientationInParentMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %location_in_child maximum allowable change
                if isfield(jointParams(ii),...
                        'location_in_child_maxParamChange')
                    if ~isempty(jointParams(ii)...
                            .location_in_child_maxParamChange)
                        LocationInChildMaxChange(ii,:)=...
                            jointParams(ii)...
                            .location_in_child_maxParamChange;
                    else
                        LocationInChildMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_child maximum allowable change
                if isfield(jointParams(ii),...
                        'orientation_in_child_maxParamChange')
                    if ~isempty(jointParams(ii)...
                            .orientation_in_child_maxParamChange)
                        OrientationInChildMaxChange(ii,:)=...
                            jointParams(ii)...
                            .orientation_in_child_maxParamChange;
                    else
                        OrientationInChildMaxChange(ii,:)=zeros(1,3);
                    end
                end
            end %end ii of for-loop
            %joint structure array (output argument 1)
            jointStruct.jointFlag=jointFlag;
            jointStruct.jointName=jointName;
            jointStruct.jointType=jointType;
            jointStruct.LocationInParent=LocationInParent;
            jointStruct.OrientationInParent=OrientationInParent;
            jointStruct.LocationInChild=LocationInChild;
            jointStruct.OrientationInChild=OrientationInChild;
            jointStruct.LocationInParentMaxChange=...
                LocationInParentMaxChange;
            jointStruct.OrientationInParentMaxChange=...
                OrientationInParentMaxChange;
            jointStruct.LocationInChildMaxChange=LocationInChildMaxChange;
            jointStruct.OrientationInChildMaxChange=...
                OrientationInChildMaxChange;
            else
                %JointParameters has not been defined in the setup file;
                %return empty variable
                jointStruct=[];
        end %end of jointParamsChk if-statement
        
        %if there is a MarkerPlateParameters tag, then extract its content
        if (plateParamsChk)
            %get field(s)
            plateParams=getfield(inputFile.startFile.objects,...
                'MarkerPlateParameters');
            %the structure may contain COMMENT and ATTRIBUTE fields; if
            %so, then remove these fields
            if (isfield(plateParams,'COMMENT'))
                plateParams=rmfield(plateParams,'COMMENT');
            end
            if (isfield(plateParams,'ATTRIBUTE'))
                plateParams=rmfield(plateParams,'ATTRIBUTE');
            end
            %see how many plateParams structure arrays are availabel
            [m,n]=size(plateParams);
            ln=max(m,n);
            %initialize following variables to avoid memory problems
            %also, by defining these parameters, if the user has not
            %specified a tag, the routine will know that the appropriate
            %tag parameters are not included in the optimization (i.e.,
            %they are set to zero)
            LocationInParent=zeros(ln,3);%location in parent (LiP)
            OrientationInParent=zeros(ln,3);%orientation in parent (OiP)
            LocationInChild=zeros(ln,3);%location in child (LiC)
            OrientationInChild=zeros(ln,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(ln,3);%LiP max change
            OrientationInParentMaxChange=zeros(ln,3);%OiP max change
            LocationInChildMaxChange=zeros(ln,3);%LiC max change
            OrientationInChildMaxChange=zeros(ln,3);%OiC max change
            %loop through and get plateParams content
            for ii=1:ln
                %plate flag
                if isfield(plateParams(ii),'apply')
                    if ~isempty(plateParams(ii).apply)
                        plateFlag(ii)={plateParams(ii).apply};
                    else
                        plateFlag(ii)={'notSpecified'};
                    end
                else
                    plateFlag={};
                end
                %plate name
                if isfield(plateParams(ii),'marker_plate_name')
                    if ~isempty(plateParams(ii).marker_plate_name)
                        plateName(ii)={plateParams(ii).marker_plate_name};
                    else
                        plateName(ii)={'notSpecified'};
                    end
                else
                    plateName={};
                end
                %plate type
                if isfield(plateParams(ii),'marker_plate_joint_type')
                    if ~isempty(plateParams(ii).marker_plate_joint_type)
                        plateType(ii)=...
                            {plateParams(ii).marker_plate_joint_type};
                    else
                        plateType(ii)={'notSpecified'};
                    end
                else
                    plateType={};
                end
                %location_in_parent
                if isfield(plateParams(ii),'location_in_parent_params')
                    if ~isempty(plateParams(ii).location_in_parent_params)
                        LocationInParent(ii,:)=...
                            plateParams(ii).location_in_parent_params;
                    else
                        LocationInParent(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_parent
                if isfield(plateParams(ii),'orientation_in_parent_params')
                    if ~isempty(plateParams(ii)...
                            .orientation_in_parent_params)
                        OrientationInParent(ii,:)=...
                            plateParams(ii).orientation_in_parent_params;
                    else
                        OrientationInParent(ii,:)=zeros(1,3);
                    end
                end
                %location_in_child
                if isfield(plateParams(ii),'location_in_child_params')
                    if ~isempty(plateParams(ii).location_in_child_params)
                        LocationInChild(ii,:)=...
                            plateParams(ii).location_in_child_params;
                    else
                        LocationInChild(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_child
                if isfield(plateParams(ii),'orientation_in_child_params')
                    if ~isempty(plateParams(ii)...
                            .orientation_in_child_params)
                        OrientationInChild(ii,:)=...
                            plateParams(ii).orientation_in_child_params;
                    else
                        OrientationInChild(ii,:)=zeros(1,3);
                    end
                end
                %location_in_parent maximum allowable change
                if isfield(plateParams(ii),...
                        'location_in_parent_maxParamChange')
                    if ~isempty(plateParams(ii)...
                            .location_in_parent_maxParamChange)
                        LocationInParentMaxChange(ii,:)=...
                            plateParams(ii)...
                            .location_in_parent_maxParamChange;
                    else
                        LocationInParentMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_parent maximum allowable change
                if isfield(plateParams(ii),...
                        'orientation_in_parent_maxParamChange')
                    if ~isempty(plateParams(ii)...
                            .orientation_in_parent_maxParamChange)
                        OrientationInParentMaxChange(ii,:)=...
                            plateParams(ii)...
                            .orientation_in_parent_maxParamChange;
                    else
                        OrientationInParentMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %location_in_child maximum allowable change
                if isfield(plateParams(ii),...
                        'location_in_child_maxParamChange')
                    if ~isempty(plateParams(ii)...
                            .location_in_child_maxParamChange)
                        LocationInChildMaxChange(ii,:)=...
                            plateParams(ii)...
                            .location_in_child_maxParamChange;
                    else
                        LocationInChildMaxChange(ii,:)=zeros(1,3);
                    end
                end
                %orientation_in_child maximum allowable change
                if isfield(plateParams(ii),...
                        'orientation_in_child_maxParamChange')
                    if ~isempty(plateParams(ii)...
                            .orientation_in_child_maxParamChange)
                        OrientationInChildMaxChange(ii,:)=...
                            plateParams(ii)...
                            .orientation_in_child_maxParamChange;
                    else
                        OrientationInChildMaxChange(ii,:)=zeros(1,3);
                    end
                end
            end %end ii of for-loop
            %marker plate structure array (output argument 2)
            plateStruct.plateFlag=plateFlag;
            plateStruct.plateName=plateName;
            plateStruct.plateJointType=plateType;
            plateStruct.LocationInParent=LocationInParent;
            plateStruct.OrientationInParent=OrientationInParent;
            plateStruct.LocationInChild=LocationInChild;
            plateStruct.OrientationInChild=OrientationInChild;
            plateStruct.LocationInParentMaxChange=...
                LocationInParentMaxChange;
            plateStruct.OrientationInParentMaxChange=...
                OrientationInParentMaxChange;
            plateStruct.LocationInChildMaxChange=...
                LocationInChildMaxChange;
            plateStruct.OrientationInChildMaxChange=...
                OrientationInChildMaxChange;
        else
            %MarkerPlateParameters has not been defined in the setup file;
            %return empty variable
            plateStruct=[];
        end %end of plateParamsChk if-statement
        
        %if there is a InputSettings tag, then extract its content
        if (inputParamsChk)
            %get field(s)
            inputParams=getfield(inputFile.startFile.objects,...
                'InputSettings');
            %the structure may contain COMMENT and ATTRIBUTE fields; if
            %so, then remove these fields
            if (isfield(inputParams,'COMMENT'))
                inputParams=rmfield(inputParams,'COMMENT');
            end
            if (isfield(inputParams,'ATTRIBUTE'))
                inputParams=rmfield(inputParams,'ATTRIBUTE');
            end
            %first take care of individual settings
            if isfield(inputParams,'input_model_file')
                if ~isempty(inputParams.input_model_file)
                    %input .osim model
                    inModel=inputParams.input_model_file;
                else
                    error('Must specify a model name!')
                end
            else
                error('Must specify a model name!')
            end
            if isfield(inputParams,'accuracy')
                if ~isempty(inputParams.accuracy)
                    %inverse kinematics solver accuracy
                    accuracy=inputParams.accuracy;
                else
                    accuracy=1e-7;%default accuracy
                end
            else
                accuracy=1e-7;%default accuracy
            end
            if isfield(inputParams,'output_model_file')
                if ~isempty(inputParams.output_model_file)
                    %optimized .osim model
                    outModel=inputParams.output_model_file;
                else
                    outModel='optModel.osim';%default output filename
                end
            else
                outModel='optModel.osim';%default output filename
            end
            if isfield(inputParams,'scale_set_file')
                if ~isempty(inputParams.scale_set_file)
                    %scale setup file
                    scaleSetFile=inputParams.scale_set_file;
                else
                    error('Must specify a scale set setup filename!')
                end
            else
                error('Must specify a scale set setup filename!')
            end
            %now take care of OptimizationGroupings tag
            if (isfield(inputParams,'OptimizationGroupings'));
                %size of OptimizationGroup
                [m,n]=size(inputParams.OptimizationGroupings...
                    .OptimizationGroup);
                ln=max(m,n);
                %loop through and get OptimizationGroup content
                %note that since there could be multiple SymmetricJointPair
                %definitions within an OptimizationGroup definition,
                %this portion of the code has been written slightly
                %different than the codes above so that the settings can be
                %saved in a structure array without causing any headaches
                for ii=1:ln
                    %experimental .trc file
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'marker_file')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).marker_file)
                            expData=inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).marker_file;
                        else
                            error('Must specify a marker filename!')
                        end
                    else
                        error('Must specify a marker filename!')
                    end
                    %experimental joint angles file (i.e., .mot)
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'coordinate_file')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).coordinate_file)
                            coordinateFile=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).coordinate_file;
                        else
                           coordinateFile={};%not specified 
                        end
                    else
                        coordinateFile={};%not specified
                    end
                    %simulation time range
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'time_range')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).time_range)
                            simTime=inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).time_range;
                        else
                            msg1='Simulation time not specified.';
                            msg2='Default range ([0 1]) will be used.';
                            fprintf(1,[msg1 ' ' msg2 '\n'])
                            simTime=[0 1];
                        end
                    else
                        msg1='Simulation time not specified.';
                        msg2='Default range ([0 1]) will be used.';
                        fprintf(1,[msg1 ' ' msg2 '\n'])
                        simTime=[0 1];
                    end
                    %flag indicating if errors should be reported
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'report_errors')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).report_errors)
                            reportErrors=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).report_errors;
                        else
                            reportErrors='false';
                        end
                    else
                        reportErrors='false';
                    end
                    %output motion file, i.e., joint angles
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'output_motion_file')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).output_motion_file)
                            outputMotionFile=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).output_motion_file;
                        else
                            outputMotionFile='ikJointAngles.mot';
                        end
                    else
                        outputMotionFile='ikJointAngles.mot';
                    end
                    %flag indicating if model marker locations should be
                    %reported
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),...
                            'report_marker_locations')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .report_marker_locations)
                            reportMarkerLocations=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .report_marker_locations;
                        else
                            reportMarkerLocations='false';
                        end
                    else
                        reportMarkerLocations='false';
                    end
                    %IK tasks setup file
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'ik_set_file')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).ik_set_file)
                            ikSetupFile=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).ik_set_file;
                        else
                            msg1='Must specify an inverse kinematics (IK)';
                            msg2='tasks setup filename.';
                            error([msg1 ' ' msg2])
                        end
                    else
                        msg1='Must specify an inverse kinematics (IK)';
                        msg2='tasks setup filename.';
                        error([msg1 ' ' msg2])
                    end
                    %optimization order
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'optimization_order')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).optimization_order)
                            optimizationOrder=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).optimization_order;
                        else
                            error('Must specify an optimization order!')
                        end
                    else
                        error('Must specify an optimization order!')
                    end
                    %which joints to include
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'WhichJoints')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).WhichJoints)
                            whichJoints=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .WhichJoints.joint_name;
                        else
                            whichJoints={};
                        end
                    else
                        whichJoints={};
                    end
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'WhichMarkerPlates')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).WhichMarkerPlates)
                            whichMarkerPlates=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .WhichMarkerPlates.marker_plate_name;
                        else
                            whichMarkerPlates={};
                        end
                    else
                        whichMarkerPlates={};
                    end
                    %penalizeChangesFlag
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'penalize_changes')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).penalize_changes)
                            penalizeChangesFlag=...
                                {inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).penalize_changes};
                        else
                            penalizeChangesFlag={'false'};
                        end
                    else
                        penalizeChangesFlag={'false'};
                    end
                    %now take care of SymmetricJointPair tag
                    %size of symmetricjointPair
                    [mm,nn]=size(inputParams.OptimizationGroupings...
                        .OptimizationGroup(ii).SymmetricJointPair);
                    ll=max(mm,nn);
                    %loop through and get SymmetricJointPair content
                    for jj=1:ll
                        %which joints are symmetric
                        if isfield(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .SymmetricJointPair(jj),'WhichJoints')
                            if ~isempty(inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj).WhichJoints)
                                %to avoid confusion with whichJoints
                                %variable, name this whichJoint
                                whichJoint(jj)=...
                                    {inputParams.OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .WhichJoints.joint_name};
                            else
                                whichJoint={};
                            end
                        else
                            whichJoint={};
                        end
                        %which dimensions to enforce symmetry
                        if isfield(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .SymmetricJointPair(jj),...
                                'enforce_joint_symmetry')
                            if ~isempty(inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .enforce_joint_symmetry)
                                enforceJointSym(jj)=...
                                    {inputParams.OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .enforce_joint_symmetry};
                            else
                                enforceJointSym={};
                            end
                        else
                            enforceJointSym={};
                        end
                        %is reflected flags
                        %translation
                        if isfield(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .SymmetricJointPair(jj),...
                                'is_reflected_translation')
                            if ~isempty(inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .is_reflected_translation)
                                isReflectedTrnFlag(jj,:)=...
                                    inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .is_reflected_translation;
                            else
                                isReflectedTrnFlag(jj,:)=zeros(1,3);
                            end
                        else
                            isReflectedTrnFlag=[];
                        end
                        %orientation
                        if isfield(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii)...
                                .SymmetricJointPair(jj),...
                                'is_reflected_orientation')
                            if ~isempty(inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .is_reflected_orientation)
                                isReflectedOrnFlag(jj,:)=...
                                    inputParams...
                                    .OptimizationGroupings...
                                    .OptimizationGroup(ii)...
                                    .SymmetricJointPair(jj)...
                                    .is_reflected_orientation;
                            else
                                isReflectedOrnFlag(jj,:)=zeros(1,3);
                            end
                        else
                            isReflectedOrnFlag=[];
                        end
                    end %end jj for-loop
                    %input settings structure array (output argument 3)
                    inputStruct.OptimizationGroup(ii).expData=expData;
                    inputStruct.OptimizationGroup(ii).coordinateFile=...
                        coordinateFile;
                    inputStruct.OptimizationGroup(ii).time_range=simTime;
                    inputStruct.OptimizationGroup(ii).report_errors=...
                        reportErrors;
                    inputStruct.OptimizationGroup(ii)...
                        .output_motion_file=outputMotionFile;
                    inputStruct.OptimizationGroup(ii)...
                        .report_marker_locations=reportMarkerLocations;
                    inputStruct.OptimizationGroup(ii)...
                        .ikSetupFile=ikSetupFile;
                    inputStruct.OptimizationGroup(ii)...
                        .optimizationOrder=optimizationOrder;
                    inputStruct.OptimizationGroup(ii)...
                        .whichJoints=whichJoints;
                    inputStruct.OptimizationGroup(ii)...
                        .whichMarkerPlates=whichMarkerPlates;
                    inputStruct.OptimizationGroup(ii)...
                        .penalizeChangesFlag=penalizeChangesFlag;
                    for kk=1:ll
                        inputStruct.OptimizationGroup(ii)...
                            .SymmetricJointPair(kk)...
                            .whichJoints=whichJoint{kk};
                        inputStruct.OptimizationGroup(ii)...
                            .SymmetricJointPair(kk)...
                            .enforceJointSym=enforceJointSym{kk};
                        if ~isempty(isReflectedTrnFlag)
                            inputStruct.OptimizationGroup(ii)...
                                .SymmetricJointPair(kk)...
                                .isReflectedTrnFlag=...
                                isReflectedTrnFlag(kk,:);
                        else
                            inputStruct.OptimizationGroup(ii)...
                                .SymmetricJointPair(kk)...
                                .isReflectedTrnFlag=isReflectedTrnFlag;
                        end
                        if ~isempty(isReflectedOrnFlag)
                            inputStruct.OptimizationGroup(ii)...
                                .SymmetricJointPair(kk)...
                                .isReflectedOrnFlag=...
                                isReflectedOrnFlag(kk,:);
                        else
                            inputStruct.OptimizationGroup(ii)...
                                .SymmetricJointPair(kk)...
                                .isReflectedOrnFlag=isReflectedOrnFlag;
                        end
                    end %end kk for-loop
                    %get other two remaining flags
                    %scaleSegments
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'scale_segments')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).scale_segments)
                            scaleSegments=...
                                {inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).scale_segments};
                        else
                            scaleSegments={'false'};
                        end
                    else
                        scaleSegments={'false'};
                    end
                    %scaleDirections
                    if isfield(inputParams.OptimizationGroupings...
                            .OptimizationGroup(ii),'scale_directions')
                        if ~isempty(inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).scale_directions)
                            scaleDirections=...
                                inputParams.OptimizationGroupings...
                                .OptimizationGroup(ii).scale_directions;
                        else
                            scaleDirections=zeros(1,3);
                        end
                    else
                        scaleDirections=[];
                    end
                    %add to structure array
                    inputStruct.OptimizationGroup(ii).scaleSegments=...
                        scaleSegments;
                    inputStruct.OptimizationGroup(ii)...
                        .scaleDirections=scaleDirections;
                end %end ii for-loop
            end
            %input settings structure array (output argument 3)
            inputStruct.inModel=inModel;
            inputStruct.accuracy=accuracy;
            inputStruct.outModel=outModel;
            inputStruct.scaleSetFile=scaleSetFile;
        else
            %InputSettings has not been defined in the setup file; return
            %empty variable
            inputStruct=[];
        end %end inputParamsChk of if-statement
        
        %prepare function output arguments
        varargout={jointStruct,plateStruct,inputStruct};
        
        end %end readInputSetupFile
        %==================================================================
        %   readInputSetupFile
        %==================================================================
        
        %==================================================================
        %   getParams
        %==================================================================
        function varargout=getParams(varargin)
        %getParams
        %
        %   getParams() extracts model parameters, i.e., design variables,
        %   as well as user-specified maximum allowable movement change
        %   using both the joint and marker plate structures, derived by
        %   reading the input setup XML file, and the .osim model.
        %   
        %   The inputs to this function are a joint structure array, a
        %   marker plate structure array, and a .osim model.
        %   
        %   The function returns the design parameters and maximum
        %   allowable movement change vectors.
        %
        %   Example function call:
        %   [v,vMax]=getParams(jointStruct,plateStruct,model);
        
        %import OpenSim Matlab API functions
        import org.opensim.modeling.*
        
        %populate variables
        jointStruct=varargin{1};%joint structure array
        markerPlateStruct=varargin{2};%marker plate structure array
        model=varargin{3};%input .osim model
        
        %setup flag to see if any of the input structures are empty (1: not
        %empty)
        jointStructFlag=1;
        markerPlateStructFlag=1;
        
        %check input structures
        if isempty(jointStruct)
            jointStructFlag=0;
        end
        if isempty(markerPlateStruct)
            markerPlateStructFlag=0;
        end
        
        %initialize variables
        v=[];%real design variable vector
        vMax=[];%maximum allowable change vector
        
        if (jointStructFlag)
            %size of jointStruct.jointName
            [m,n]=size(jointStruct.jointName);
            %initialize variables
            LocationInParent=zeros(n,3);%location in parent (LiP)
            OrientationInParent=zeros(n,3);%orientation in parent (OiP)
            LocationInChild=zeros(n,3);%location in child (LiC)
            OrientationInChild=zeros(n,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(n,3);%location in parent (LiP)
            OrientationInParentMaxChange=...
                zeros(n,3);%orientation in parent (OiP)
            LocationInChildMaxChange=...
                zeros(n,3);%location in child (LiC)
            OrientationInChildMaxChange=...
                zeros(n,3);%orientation in child (OiC)
            %loop through parameters and extract values
            for ii=1:n
                %check location_in_parent
                if sum(jointStruct.LocationInParent(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getLocationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_parent so that Matlab can read/use
                    %it
                    LocationInParent(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v LocationInParent(ii,logical...
                        (jointStruct.LocationInParent(ii,:)))];
                    %populate maximum allowable change vector
                    LocationInParentMaxChange(ii,:)=...
                        jointStruct.LocationInParentMaxChange(ii,:);
                    vMax=[vMax LocationInParentMaxChange(ii,...
                        logical(jointStruct...
                        .LocationInParentMaxChange(ii,:)))];
                end
                %check orientation_in_parent
                if sum(jointStruct.OrientationInParent(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get orientation_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getOrientationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_parent so that Matlab can
                    %read/use it
                    OrientationInParent(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v OrientationInParent(ii,logical(jointStruct...
                        .OrientationInParent(ii,:)))];
                    %populate maximum allowable change vector
                    OrientationInParentMaxChange(ii,:)=jointStruct...
                        .OrientationInParentMaxChange(ii,:)*(pi/180);%rad
                    vMax=[vMax OrientationInParentMaxChange(ii,...
                        logical(jointStruct...
                        .OrientationInParentMaxChange(ii,:)))];
                end
                %check location_in_child
                if sum(jointStruct.LocationInChild(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_child
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getLocation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_child so that Matlab can read/use it
                    LocationInChild(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v LocationInChild(ii,...
                        logical(jointStruct.LocationInChild(ii,:)))];
                    %populate maximum allowable change vector
                    LocationInChildMaxChange(ii,:)=jointStruct...
                        .LocationInChildMaxChange(ii,:);
                    vMax=[vMax LocationInChildMaxChange(ii,...
                        logical(jointStruct...
                        .LocationInChildMaxChange(ii,:)))];
                end
                %check orientation_in_child
                if sum(jointStruct.OrientationInChild(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getOrientation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_child so that Matlab can read/use
                    %it
                    OrientationInChild(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v OrientationInChild(ii,logical(jointStruct...
                        .OrientationInChild(ii,:)))];
                    %populate maximum allowable change vector
                    OrientationInChildMaxChange(ii,:)=jointStruct...
                        .OrientationInChildMaxChange(ii,:)*(pi/180);%rad
                    vMax=[vMax OrientationInChildMaxChange(ii,...
                        logical(jointStruct...
                        .OrientationInChildMaxChange(ii,:)))];
                end
            end %end of for-loop
        end %end jointStructFlag if-statement
        
        if (markerPlateStructFlag)
            %size of markerPlateStruct.plateName
            [m,n]=size(markerPlateStruct.plateName);
            %initialize variables
            LocationInParent=zeros(n,3);%location in parent (LiP)
            OrientationInParent=zeros(n,3);%orientation in parent (OiP)
            LocationInChild=zeros(n,3);%location in child (LiC)
            OrientationInChild=zeros(n,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(n,3);%location in parent (LiP)
            OrientationInParentMaxChange=...
                zeros(n,3);%orientation in parent (OiP)
            LocationInChildMaxChange=zeros(n,3);%location in child (LiC)
            OrientationInChildMaxChange=...
                zeros(n,3);%orientation in child (OiC)
            %loop through parameters and extract values
            for ii=1:n
                %check location_in_parent
                if sum(markerPlateStruct.LocationInParent(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii})...
                        .getLocationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_parent so that Matlab can read/use
                    %it
                    LocationInParent(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v LocationInParent(ii,...
                        logical(markerPlateStruct...
                        .LocationInParent(ii,:)))];
                    %populate maximum allowable change vector
                    LocationInParentMaxChange(ii,:)=markerPlateStruct...
                        .LocationInParentMaxChange(ii,:);
                    vMax=[vMax LocationInParentMaxChange(ii,...
                        logical(markerPlateStruct...
                        .LocationInParentMaxChange(ii,:)))];
                end
                %check orientation_in_parent
                if sum(markerPlateStruct.OrientationInParent(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get orientation_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii})...
                        .getOrientationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_parent so that Matlab can
                    %read/use it
                    OrientationInParent(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v OrientationInParent(ii,...
                        logical(markerPlateStruct...
                        .OrientationInParent(ii,:)))];
                    %populate maximum allowable change vector
                    OrientationInParentMaxChange(ii,:)=...
                        markerPlateStruct...
                        .OrientationInParentMaxChange(ii,:)*(pi/180);%rad
                    vMax=[vMax OrientationInParentMaxChange(ii,...
                        logical(markerPlateStruct...
                        .OrientationInParentMaxChange(ii,:)))];
                end
                %check location_in_child
                if sum(markerPlateStruct.LocationInChild(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_child
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii}).getLocation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_child so that Matlab can read/use it
                    LocationInChild(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v LocationInChild(ii,...
                        logical(markerPlateStruct.LocationInChild(ii,:)))];
                    %populate maximum allowable change vector
                    LocationInChildMaxChange(ii,:)=markerPlateStruct...
                        .LocationInChildMaxChange(ii,:);
                    vMax=[vMax LocationInChildMaxChange(ii,...
                        logical(markerPlateStruct...
                        .LocationInChildMaxChange(ii,:)))];
                end
                %check orientation_in_child
                if sum(markerPlateStruct.OrientationInChild(ii,:))>=1
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii})...
                        .getOrientationInChild(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_child so that Matlab can read/use
                    %it
                    OrientationInChild(ii,:)=str2num(OpSimADloc);
                    %populate real design variable vector
                    v=[v OrientationInChild(ii,logical(markerPlateStruct...
                        .OrientationInChild(ii,:)))];
                    %populate maximum allowable change vector
                    OrientationInChildMaxChange(ii,:)=...
                        markerPlateStruct...
                        .OrientationInChildMaxChange(ii,:)*(pi/180);%rad
                    vMax=[vMax OrientationInChildMaxChange(ii,...
                        logical(markerPlateStruct...
                        .OrientationInChildMaxChange(ii,:)))];
                end
            end %end of for-loop
        end %end markerPlateStructFlag if-statement
        
        %make sure output variables are in column vector format
        v=v(:);
        vMax=vMax(:);
        
        %prepare function output arguments
        varargout={v,vMax};
        
        end %end getParams
        %==================================================================
        %   getParams
        %==================================================================
        
        %==================================================================
        %   setParams
        %==================================================================
        function setParams(varargin)
        %setParams
        %
        %   setParams() modifies the .osim model parameters.
        %   
        %   The input arguments to this function are the joint and marker
        %   plate structure arrays, the .osim model, the design parameters,
        %   and the maximum allowable movement change vectors.
        %   
        %   The function does not return an output argument.
        %   
        %   Example function call:
        %   setParams(jointStructOpt,plateStructOpt,model,v,dv);
        
        %import OpenSim Matlab API functions
        import org.opensim.modeling.*
        
        %populate variables
        jointStruct=varargin{1};%joint structure array
        markerPlateStruct=varargin{2};%marker plate structure array
        model=varargin{3};%input .osim model
        v=varargin{4};%design parameters
        dv=varargin{5};%maximum allowable changes
        
        %setup flag to see if any of the input structures are empty (1:not
        %empty)
        jointStructFlag=1;
        markerPlateStructFlag=1;
        
        %check input structures
        if isempty(jointStruct)
            jointStructFlag=0;
        end
        if isempty(markerPlateStruct)
            markerPlateStructFlag=0;
        end
        
        %initialize variables
        indexLocation=[];
        count=0;
        
        if (jointStructFlag)
            %size of jointStruct.jointName
            [m,n]=size(jointStruct.jointName);
            %initialize variables
            LocationInParent=zeros(n,3);%location in parent (LiP)
            OrientationInParent=zeros(n,3);%orientation in parent (OiP)
            LocationInChild=zeros(n,3);%location in child (LiC)
            OrientationInChild=zeros(n,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(n,3);%location in parent (LiP)
            OrientationInParentMaxChange=...
                zeros(n,3);%orientation in parent (OiP)
            LocationInChildMaxChange=zeros(n,3);%location in child (LiC)
            OrientationInChildMaxChange=...
                zeros(n,3);%orientation in child (OiC)
            %loop through parameters and extract values
            for ii=1:n
                %check location_in_parent
                if sum(jointStruct.LocationInParent(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(jointStruct.jointName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getLocationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_parent so that Matlab can read/use
                    %it
                    LocationInParent(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros(jointStruct...
                        .LocationInParent(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of location_in_parent is to
                    %be modified by calling local function 
                    %utilFunctions.getCaseNumber()
                    caseNumber=utilFunctions.getCaseNumber(jointStruct...
                        .LocationInParent(ii,:));
                    %modify location_in_parent by calling local function
                    %utilFunctions.modifyVariable
                    modLocation=utilFunctions.modifyVariable...
                        (LocationInParent(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified location in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modLocation);
                    %update location_in_parent in OpenSim model
                    currentJoint.set_location_in_parent(OpSimADmodLoc);
                    currentJoint.upd_location_in_parent;
                end
                %check orientation_in_parent
                if sum(jointStruct.OrientationInParent(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet().get(jointStruct...
                        .jointName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get orientation_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getOrientationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_parent so that Matlab can
                    %read/use it
                    OrientationInParent(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros(jointStruct...
                        .OrientationInParent(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of orientation_in_parent is
                    %to be modified by calling local function 
                    %utilFunctions.getCaseNumber
                    caseNumber=utilFunctions.getCaseNumber...
                        (jointStruct.OrientationInParent(ii,:));
                    %modify location_in_parent by calling local function
                    %utilFunctions.modifyVariable
                    modOrientation=utilFunctions.modifyVariable...
                        (OrientationInParent(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified orientation in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modOrientation);
                    %update orientation_in_parent in OpenSim model
                    currentJoint.set_orientation_in_parent(OpSimADmodLoc);
                    currentJoint.upd_orientation_in_parent;
                end
                %check location_in_child
                if sum(jointStruct.LocationInChild(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(jointStruct.jointName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_child
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getLocation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_child so that Matlab can read/use it
                    LocationInChild(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros(jointStruct...
                        .LocationInChild(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of location_in_child is to
                    %be modified by calling local function 
                    %utilFunctions.getCaseNumber
                    caseNumber=utilFunctions.getCaseNumber(jointStruct...
                        .LocationInChild(ii,:));
                    %modify location_in_child by calling local function
                    %utilFunctions.modifyVariable
                    modLocation=utilFunctions.modifyVariable...
                        (LocationInChild(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified location in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modLocation);
                    %update location_in_child in OpenSim model
                    currentJoint.set_location(OpSimADmodLoc);
                    currentJoint.upd_location;
                end
                %check orientation_in_child
                if sum(jointStruct.OrientationInChild(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet().get(jointStruct...
                        .jointName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(jointStruct.jointName{ii})...
                        .getOrientation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_child so that Matlab can read/use
                    %it
                    OrientationInChild(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros(jointStruct...
                        .OrientationInChild(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of orientation_in_child is
                    %to be modified by calling local function 
                    %utilFunctions.getCaseNumber
                    caseNumber=utilFunctions.getCaseNumber(jointStruct...
                        .OrientationInChild(ii,:));
                    %modify orientation_in_child by calling local function
                    %utilFunctions.modifyVariable
                    modOrientation=utilFunctions.modifyVariable...
                        (OrientationInChild(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified orientation in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modOrientation);
                    %update orientation_in_child in OpenSim model
                    currentJoint.set_orientation(OpSimADmodLoc);
                    currentJoint.upd_orientation;
                end
            end %end ii for-loop
        end %end jointStructFlag if-statemnt
        
        if (markerPlateStructFlag)
            %size of markerPlateStruct.plateName
            [m,n]=size(markerPlateStruct.plateName);
            %initialize variables
            LocationInParent=zeros(n,3);%location in parent (LiP)
            OrientationInParent=zeros(n,3);%orientation in parent (OiP)
            LocationInChild=zeros(n,3);%location in child (LiC)
            OrientationInChild=zeros(n,3);%orientation in child (OiC)
            LocationInParentMaxChange=zeros(n,3);%location in parent (LiP)
            OrientationInParentMaxChange=...
                zeros(n,3);%orientation in parent (OiP)
            LocationInChildMaxChange=zeros(n,3);%location in child (LiC)
            OrientationInChildMaxChange=...
                zeros(n,3);%orientation in child (OiC)
            %loop through parameters and extract values
            for ii=1:n
                %check location_in_parent
                if sum(markerPlateStruct.LocationInParent(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(markerPlateStruct.plateName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii})...
                        .getLocationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_parent so that Matlab can read/use
                    %it
                    LocationInParent(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros...
                        (markerPlateStruct.LocationInParent(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of location_in_parent is to
                    %be modified by calling local function 
                    %utilFunctions.getCaseNumber
                    caseNumber=utilFunctions.getCaseNumber...
                        (markerPlateStruct.LocationInParent(ii,:));
                    %modify location_in_parent by calling local function
                    %utilFunctions.modifyVariable
                    modLocation=utilFunctions.modifyVariable...
                        (LocationInParent(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified location in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modLocation);
                    %update location_in_parent in OpenSim model
                    currentJoint.set_location_in_parent(OpSimADmodLoc);
                    currentJoint.upd_location_in_parent;
                end
                %check orientation_in_parent
                if sum(markerPlateStruct.OrientationInParent(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(markerPlateStruct.plateName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get orientation_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii})...
                        .getOrientationInParent(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_parent so that Matlab can
                    %read/use it
                    OrientationInParent(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros...
                        (markerPlateStruct...
                        .OrientationInParent(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of orientation_in_parent is
                    %to be modified by calling local function 
                    %utilFunctions.getCaseNumber()
                    caseNumber=utilFunctions.getCaseNumber...
                        (markerPlateStruct.OrientationInParent(ii,:));
                    %modify orientation_in_parent by calling local function
                    %utilFunctions.modifyVariable
                    modOrientation=utilFunctions.modifyVariable...
                        (OrientationInParent(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified orientation in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modOrientation);
                    %update orientation_in_parent in OpenSim model
                    currentJoint.set_orientation_in_parent(OpSimADmodLoc);
                    currentJoint.upd_orientation_in_parent;
                end
                %check location_in_child
                if sum(markerPlateStruct.LocationInChild(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(markerPlateStruct.plateName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_child
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii}).getLocation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store location_in_child so that Matlab can read/use it
                    LocationInChild(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros...
                        (markerPlateStruct.LocationInChild(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of location_in_child is to
                    %be modified by calling local function 
                    %utilFunctions.getCaseNumber()
                    caseNumber=utilFunctions.getCaseNumber...
                        (markerPlateStruct.LocationInChild(ii,:));
                    %modify location_in_child by calling local function
                    %utilFunctions.modifyVariable
                    modLocation=utilFunctions.modifyVariable...
                        (LocationInChild(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified location in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modLocation);
                    %update location_in_child in OpenSim model
                    currentJoint.set_location(OpSimADmodLoc);
                    currentJoint.upd_location;
                end
                %check orientation_in_child
                if sum(markerPlateStruct.OrientationInChild(ii,:))>=1
                    %for simplicity, get model joint and store it for later
                    %use
                    currentJoint=model.updJointSet()...
                        .get(markerPlateStruct.plateName{ii});
                    %initialize OpenSim ArrayDouble variable
                    OpSimADinitLoc=ArrayDouble.createVec3([0.0,0.0,0.0]);
                    %get location_in_parent
                    model.getJointSet.get(markerPlateStruct...
                        .plateName{ii}).getOrientation(OpSimADinitLoc);
                    OpSimADloc=ArrayDouble...
                        .getValuesFromVec3(OpSimADinitLoc);
                    %store orientation_in_child so that Matlab can read/use
                    %it
                    OrientationInChild(ii,:)=str2num(OpSimADloc);
                    %get index location in v
                    indexLocation=(count+1):(length(nonzeros...
                        (markerPlateStruct...
                        .OrientationInChild(ii,:)))+count);
                    count=count+length(indexLocation);
                    %determine which component of orientation_in_child is
                    %to be modified by calling local function 
                    %utilFunctions.getCaseNumber()
                    caseNumber=utilFunctions.getCaseNumber...
                        (markerPlateStruct.OrientationInChild(ii,:));
                    %modify orientation_in_child by calling local function
                    %utilFunctions.modifyVariable
                    modOrientation=utilFunctions.modifyVariable...
                        (OrientationInChild(ii,:),caseNumber,...
                        indexLocation,v,dv);
                    %copy modified orientation in OpenSim ArrayDouble
                    OpSimADmodLoc=ArrayDouble.createVec3(modOrientation);
                    %update orientation_in_child in OpenSim model
                    currentJoint.set_orientation(OpSimADmodLoc);
                    currentJoint.upd_orientation;
                end
            end %end ii for-loop
        end %end markerPlateStructFlag if-statement
        
        end %end setParams
        %==================================================================
        %   setParams
        %==================================================================
        
        %==================================================================
        %   getCaseNumber
        %==================================================================
        function varargout=getCaseNumber(varargin)
        %getCaseNumber
        %
        %   getCaseNumber() finds the appropriate case number associated
        %   with the selected model parameter(s) by referencing a lookup
        %   table (LUT).
        %   
        %   The input to this function is the vector containing the
        %   selected model parameters--in the form of 1s and 0s. 
        %   
        %   The function returns the assigned case number vector.
        %   
        %   Example function call:
        %   caseNumber=getCaseNumber(jS.LocationInParent(index,:));
        
        %populate variables
        inVector=varargin{1};%input vector
        
        %set some parameters
        numOfParams=3;%number of parameters
        ii0=2;
        numOfEntries=2^numOfParams;%number of entries
        LUT=false(numOfEntries,numOfParams);%initaize lookup table
        
        for jj=1:numOfParams
            jj0=2^jj;%number of entries
            jj1=(jj0/2)-1;
            ii1=numOfParams-jj+1;
            for ii0=ii0:jj0:numOfEntries
                for kk=0:jj1
                    LUT(ii0+kk,ii1)=true;%populate table
                end
            end
            ii0=jj0+1;
        end %end jj for-loop
        
        indexConvention=[1 2 3];%defined index convention
        %convert LUT into index convention
        [r,c]=size(LUT);
        N=repmat(indexConvention,r,1);
        LUT=LUT.*N;%convert LUT
        
        %convert input argument to index convention
        outVector=inVector.*indexConvention;
        
        %find location of outVector in LUT
        [~,caseNumber]=ismember(outVector,LUT,'rows');
        
        %prepare function output arguments
        varargout={caseNumber};
        
        end %end getCaseNumber
        %==================================================================
        %   getCaseNumber
        %==================================================================
        
        %==================================================================
        %   modifyVariable
        %==================================================================
        function varargout=modifyVariable(varargin)
        %modifyVariable
        %
        %   modifyVariable() modifies appropriate variable(s) based on the
        %   derived case number obtained from getCaseNumber() routine.
        %   
        %   The input arguments to this function are the current variables,
        %   case number, design variable index location, design
        %   parameter(s), and vector of maximum allowable movement change.
        %   
        %   The function returns the vector of modified variable(s).
        %   
        %   Example function call:
        %   modVar=modifyVariable(currentVar(index,:),caseNumber,
        %                         indexLocation,v,dv);
        
        %populate variables
        oldLocation=varargin{1};%old location
        caseNumber=varargin{2};%case number
        indexLocation=varargin{3};%index location
        v=varargin{4};%design parameters
        dv=varargin{5};%maximum allowable changes
        
        switch caseNumber
            case 1
                newLocation=[oldLocation(1)...
                    oldLocation(2) oldLocation(3)];
            case 2
                newLocation=[oldLocation(1)...
                    oldLocation(2) v(indexLocation)+dv(indexLocation)];
            case 3
                newLocation=[oldLocation(1)...
                    v(indexLocation)+dv(indexLocation) oldLocation(3)];
            case 4
                newLocation=[oldLocation(1)...
                    v(indexLocation(1))+dv(indexLocation(1))...
                    v(indexLocation(2))+dv(indexLocation(2))];
            case 5
                newLocation=[v(indexLocation)+dv(indexLocation)...
                    oldLocation(2) oldLocation(3)];
            case 6
                newLocation=[v(indexLocation(1))+dv(indexLocation(1))...
                    oldLocation(2)...
                    v(indexLocation(2))+dv(indexLocation(2))];
            case 7
                newLocation=[v(indexLocation(1))+dv(indexLocation(1))...
                    v(indexLocation(2))+dv(indexLocation(2))...
                    oldLocation(3)];
            case 8
                newLocation=[v(indexLocation(1))+dv(indexLocation(1))...
                    v(indexLocation(2))+dv(indexLocation(2))...
                    v(indexLocation(3))+dv(indexLocation(3))];
        end %end of switch
        
        %prepare function output arguments
        varargout={newLocation};
        
        end %end modifyVariable
        %==================================================================
        %   modifyVariable
        %==================================================================
        
        %==================================================================
        %   calcMarkerErrors
        %==================================================================
        function varargout=calcMarkerErrors(varargin)
        %calcMarkerErrors
        %
        %   calcMarkerErrors() computes the time-varying error of each
        %   marker in the model compared to the experimental data.
        %   
        %   The input arguments to this function are the .osim model, the
        %   inverse kinematics solver object, sampling time (sec), number
        %   of frames, number of markers in the model, and simulation start
        %   time (sec).
        %   
        %   The function returns the marker errors, virutal marker
        %   locations, and marker names.
        %   
        %   Example function call:
        %   [markerErr,markerPos,markerNames]=calcMarkerErrors(model,
        %                                                      ikSolver,dt,
        %                                                      Nframes,
        %                                                      Nmarkers,
        %                                                      startTime);

        %import OpenSim Matlab API functions
        import org.opensim.modeling.*
        
        %populate variables
        model=varargin{1};%.osim model structure
        ikSolver=varargin{2};%ikSolver structure
        dt=varargin{3};%sampling time (sec)
        Nframes=varargin{4};%number of frames
        nm=varargin{5};%number of markers
        startTime=varargin{6};%simulation start time (sec)
        
        %initialize OpenSim model
        state=model.initSystem;
        
        %set simulation start time
        state.setTime(startTime);
        
        %assemble
        ikSolver.assemble(state);
        
        %initialize variables
        markerError=zeros(floor(Nframes),nm);
        locations=zeros(nm,3);
        markerLocations=zeros(floor(Nframes-1),3*nm);
        
        %run main loop
        for ii=1:Nframes-1
            state.setTime(startTime+ii*dt);
            ikSolver.track(state);
            for jj=0:nm-1
                %compute marker location
                markerError(ii+1,jj+1)=ikSolver...
                    .computeCurrentMarkerError(jj);
                %compute marker location
                currentLocation=ikSolver.computeCurrentMarkerLocation(jj);
                %get marker location from Vec3
                locations(jj+1,1)=currentLocation.get(0);
                locations(jj+1,2)=currentLocation.get(1);
                locations(jj+1,3)=currentLocation.get(2);
            end
            %store values
            markerLocations(ii,:)=reshape(locations',[1 numel(locations)]);
        end %end ii for-loop
        
        %create cell array for marker names
        markerNames=cell(1,nm);
        
        %get marker names
        for jj=0:nm-1
            markerNames{jj+1}=char(ikSolver.getMarkerNameForIndex(jj));
        end
        
        %prepare function output arguments
        varargout={markerError,markerLocations,markerNames};
        
        end %end calcMarkerErrors
        %==================================================================
        %   calcMarkerErrors
        %==================================================================
        
        %==================================================================
        %   initOptStruct
        %==================================================================
        function varargout=initOptStruct(varargin)
        %initOptStruct
        %
        %   initOptStruct() initializes optimization specific
        %   parameters.
        %   
        %   The input arguments to this function are the joint
        %   and marker plate structures, the number of joints and number of
        %   marker plates, and the joints and marker plates that are
        %   included in the optimization (i.e., whichJoints and
        %   whichMarkerPlates).
        %   
        %   The function returns the optimization specific joint and marker
        %   plate structure arrays, respectively.
        %   
        %   Example function call:
        %   [jSOpt,mSOpt]=initOptStruct(jS,mS,numOfJoints,
        %                               numOfMarkerPlates,whichJoints,
        %                               whichMarkerPlates);
        
        %populate variables
        jointStruct=varargin{1};%joints structure array
        markerPlateStruct=varargin{2};%marker plates structure array
        numOfJoints=varargin{3};%number of joints
        numOfPlates=varargin{4};%number of marker plates
        whichJoints=varargin{5};%which joints are included
        whichMarkerPlates=varargin{6};%which marker plates are included
        
        %initialize optimization specific joint structure
        jointStructOpt.jointName=cell(1,numOfJoints);
        jointStructOpt.jointType=cell(1,numOfJoints);
        jointStructOpt.LocationInParent=zeros(numOfJoints,3);
        jointStructOpt.OrientationInParent=zeros(numOfJoints,3);
        jointStructOpt.LocationInChild=zeros(numOfJoints,3);
        jointStructOpt.OrientationInChild=zeros(numOfJoints,3);
        jointStructOpt.LocationInParent=zeros(numOfJoints,3);
        jointStructOpt.OrientationInParent=zeros(numOfJoints,3);
        jointStructOpt.LocationInChild=zeros(numOfJoints,3);
        jointStructOpt.OrientationInChild=zeros(numOfJoints,3);
        
        %initialize optimization specific marker plate structure
        markerPlateStructOpt.plateName=cell(1,numOfPlates);
        markerPlateStructOpt.plateJointType=cell(1,numOfPlates);
        markerPlateStructOpt.LocationInParent=zeros(numOfPlates,3);
        markerPlateStructOpt.OrientationInParent=zeros(numOfPlates,3);
        markerPlateStructOpt.LocationInChild=zeros(numOfPlates,3);
        markerPlateStructOpt.OrientationInChild=zeros(numOfPlates,3);
        markerPlateStructOpt.LocationInParent=zeros(numOfPlates,3);
        markerPlateStructOpt.OrientationInParent=zeros(numOfPlates,3);
        markerPlateStructOpt.LocationInChild=zeros(numOfPlates,3);
        markerPlateStructOpt.OrientationInChild=zeros(numOfPlates,3);
        
        %create optimization specific joint and plate structures
        for jj=1:numOfJoints
            %determine index of joint in general joint structure
            JointIndex=strcmp(jointStruct.jointName,whichJoints{jj});
            if isempty(JointIndex)
                msg1='Joint definition error. ';
                msg2='Make sure all joints in optimization group are ';
                msg3='correctly defined in input setup file.';
                error([msg1 msg2 msg3])
            end
            %assign parameters to optimization specific structure
            jointStructOpt.jointName(jj)=jointStruct.jointName(JointIndex);
            jointStructOpt.jointType(jj)=jointStruct.jointType(JointIndex);
            jointStructOpt.LocationInParent(jj,:)=jointStruct...
                .LocationInParent(JointIndex,:);
            jointStructOpt.OrientationInParent(jj,:)=jointStruct...
                .OrientationInParent(JointIndex,:);
            jointStructOpt.LocationInChild(jj,:)=jointStruct...
                .LocationInChild(JointIndex,:);
            jointStructOpt.OrientationInChild(jj,:)=jointStruct...
                .OrientationInChild(JointIndex,:);
            jointStructOpt.LocationInParentMaxChange(jj,:)=jointStruct...
                .LocationInParentMaxChange(JointIndex,:);
            jointStructOpt.OrientationInParentMaxChange(jj,:)=...
                jointStruct.OrientationInParentMaxChange(JointIndex,:);
            jointStructOpt.LocationInChildMaxChange(jj,:)=...
                jointStruct.LocationInChildMaxChange(JointIndex,:);
            jointStructOpt.OrientationInChildMaxChange(jj,:)=...
                jointStruct.OrientationInChildMaxChange(JointIndex,:);
        end %end jj for-loop
        
        for jj=1:numOfPlates
            %determine index of marker plate in general marker plate
            %structure
            PlateIndex=strcmp(markerPlateStruct...
                .plateName,whichMarkerPlates{jj});
            if isempty(PlateIndex)
                msg1='Marker plate definition error. ';
                msg2='Make sure all marker plates in optimization group ';
                msg3='are correctly defined in input setup file.';
                error([msg1 msg2 msg3])
            end
            %assign parameters to optimization specific structure
            markerPlateStructOpt.plateName(jj)=...
                markerPlateStruct.plateName(PlateIndex);
            markerPlateStructOpt.plateJointType(jj)=...
                markerPlateStruct.plateJointType(PlateIndex);
            markerPlateStructOpt.LocationInParent(jj,:)=...
                markerPlateStruct.LocationInParent(PlateIndex,:);
            markerPlateStructOpt.OrientationInParent(jj,:)=...
                markerPlateStruct.OrientationInParent(PlateIndex,:);
            markerPlateStructOpt.LocationInChild(jj,:)=...
                markerPlateStruct.LocationInChild(PlateIndex,:);
            markerPlateStructOpt.OrientationInChild(jj,:)=...
                markerPlateStruct.OrientationInChild(PlateIndex,:);
            markerPlateStructOpt.LocationInParentMaxChange(jj,:)=...
                markerPlateStruct.LocationInParentMaxChange(PlateIndex,:);
            markerPlateStructOpt.OrientationInParentMaxChange(jj,:)=...
                markerPlateStruct...
                .OrientationInParentMaxChange(PlateIndex,:);
            markerPlateStructOpt.LocationInChildMaxChange(jj,:)=...
                markerPlateStruct.LocationInChildMaxChange(PlateIndex,:);
            markerPlateStructOpt.OrientationInChildMaxChange(jj,:)=...
                markerPlateStruct...
                .OrientationInChildMaxChange(PlateIndex,:);
        end %end jj for-loop
        
        %prepare function output arguments
        varargout={jointStructOpt,markerPlateStructOpt};
        
        end %end initOptStruct
        %==================================================================
        %   initOptStruct
        %==================================================================
        
                %==================================================================
        %   buildSymConstraints
        %==================================================================
        function varargout=buildSymConstraints(varargin)
        %buildSymConstraints
        %
        %   buildSymConstraints() builds symmetry constraints and passes
        %   them to the main kinematic calibration function.
        %   
        %   The input arguments to this function are the optimization
        %   grouping structure array and the optimization specific joint
        %   structure array (not to be confused with the input joint
        %   structure array).
        %   
        %   The function returns the index pairs for the constrained
        %   values, for translations, orientations, or both.
        %   
        %   Example function call for translation indices:
        %   cnstrPairIndTrn=buildSymConstraints(optGroupStruct,
        %                                       jointStructOpt,
        %                                       numOfJointParams);
        
        %populate variables
        optGroup=varargin{1};%optimization group structure array
        %optimization specific joint structure array
        jointStructOpt=varargin{2};
        numOfJointParams=varargin{3};%num of joint parameters
        
        %define a new variable for ease of referencing
        symmetricJointPair=optGroup.SymmetricJointPair;
        
        %determine number of symmetry constraints
        numSym=numel(symmetricJointPair);
        symJointIndex=zeros(numSym,2);
        
        for jj=1:numSym
            %define a new variable for ease of referencing
            jointPair=symmetricJointPair(jj);
            
            %find indices of constrained joints
            for kk=1:2
                [junk,symJointIndex(jj,kk)]=max(strcmp(jointStructOpt...
                    .jointName,jointPair.whichJoints{kk}));
            end
            
            %initialize matrices that store constraint pair indices
            %translation
            constrainTX=zeros(2,2);
            constrainTY=zeros(2,2);
            constrainTZ=zeros(2,2);
            
            %orientation
            constrainRX=zeros(2,2);
            constrainRY=zeros(2,2);
            constrainRZ=zeros(2,2);
            
            %determine if x-direction is being constrained (translation)
            if max(strcmp(lower(jointPair.enforceJointSym),'tx'))
                count=1;
                %determine indices in design parameter vector (v) that will
                %be constrained; indices are stored in constrainTX
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of location_in_parent_parameter for
                    %x-direction
                    if jointStructOpt.LocationInParent(index,1)
                        indexCountParent=jointStructOpt...
                            .LocationInParent(index,1);
                        %store index: parent on top, child on bottom
                        constrainTX(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of location_in_child_parameter for
                    %x-direction
                    if jointStructOpt.LocationInChild(index,1)
                        indexCountChild=sum([...
                            jointStructOpt.LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,1)]);
                        %store index: parent on top, child on bottom
                        constrainTX(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end %end kk for-loop
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedTrnFlag(1)
                    constrainTX(:,2)=-constrainTX(:,2);
                end
            end %end if-statement for x-direction
            
            %determine if y-direction is being constrained (translation)
            if max(strcmp(lower(jointPair.enforceJointSym),'ty'))
                count=1;
                %determine indices in design parameter vector (v) that will
                %be constrained; indices are stored in constrainTY
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of location_in_parent_parameter for
                    %y-direction
                    if jointStructOpt.LocationInParent(index,2)
                        indexCountParent=sum(jointStructOpt...
                            .LocationInParent(index,1:2));
                        %store index: parent on top, child on bottom
                        constrainTY(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of location_in_child_parameter for
                    %y-direction
                    if jointStructOpt.LocationInChild(index,2)
                        indexCountChild=sum([jointStructOpt...
                            .LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,1:2)]);
                        %store index: parent on top, child on bottom
                        constrainTY(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedTrnFlag(2)
                    constrainTY(:,2)=-constrainTY(:,2);
                end
            end %end if-statement for y-direction
            
            %determine if z-direction is being constrained (translation)
            if max(strcmp(lower(jointPair.enforceJointSym),'tz'))
                count=1;
                %determine number of values in v before the parameters
                %in this vector are listed
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of location_in_parent_parameter for
                    %z-direction
                    if jointStructOpt.LocationInParent(index,3)
                        indexCountParent=sum(jointStructOpt...
                            .LocationInParent(index,:));
                        %store index: parent on top, child on bottom
                        constrainTZ(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of location_in_child_parameter for
                    %z-direction
                    if jointStructOpt.LocationInChild(index,3)
                        indexCountChild=sum([jointStructOpt...
                            .LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,:)]);
                        %store index: parent on top, child on bottom
                        constrainTZ(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedTrnFlag(3)
                    constrainTZ(:,2)=-constrainTZ(:,2);
                end
            end %end if-statement for z-direction
            
            %determine if x-direction is being constrained (orientation)
            if max(strcmp(lower(jointPair.enforceJointSym),'rx'))
                count=1;
                %determine indices in design parameter vector (v) that will
                %be constrained; indices are stored in constrainTX
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of orientation_in_parent_parameter for
                    %x-direction
                    if jointStructOpt.OrientationInParent(index,1)
                        indexCountParent=sum([...
                            jointStructOpt.LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,1)]);
                        %store index: parent on top, child on bottom
                        constrainRX(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of orientation_in_child_parameter for
                    %x-direction
                    if jointStructOpt.OrientationInChild(index,1)
                        indexCountChild=sum([...
                            jointStructOpt.LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,:)...
                            jointStructOpt.OrientationInChild(index,1)]);
                        %store index: parent on top, child on bottom
                        constrainRX(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end %end kk for-loop
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedOrnFlag(1)
                    constrainRX(:,2)=-constrainRX(:,2);
                end
            end %end if-statement for x-direction
            
            %determine if y-direction is being constrained (translation)
            if max(strcmp(lower(jointPair.enforceJointSym),'ry'))
                count=1;
                %determine indices in design parameter vector (v) that will
                %be constrained; indices are stored in constrainTY
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of orientation_in_parent_parameter for
                    %y-direction
                    if jointStructOpt.OrientationInParent(index,2)
                        indexCountParent=sum([...
                            jointStructOpt.LocationInParent(index,:)...
                            jointStructOpt...
                            .OrientationInParent(index,1:2)]);
                        %store index: parent on top, child on bottom
                        constrainRY(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of orientation_in_child_parameter for
                    %y-direction
                    if jointStructOpt.OrientationInChild(index,2)
                        indexCountChild=sum([jointStructOpt...
                            .LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,:)...
                            jointStructOpt.OrientationInChild(index,1:2)]);
                        %store index: parent on top, child on bottom
                        constrainRY(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedOrnFlag(2)
                    constrainRY(:,2)=-constrainRY(:,2);
                end
            end %end if-statement for y-direction
            
            %determine if z-direction is being constrained (translation)
            if max(strcmp(lower(jointPair.enforceJointSym),'rz'))
                count=1;
                %determine number of values in v before the parameters
                %in this vector are listed
                for kk=1:2
                    %assign index of joint being constrained
                    index=symJointIndex(jj,kk);
                    %determine number of values in v before the parameters
                    %in this vector are listed
                    if index==1
                        numParamsBefore=0;
                    else
                        numParamsBefore=sum(numOfJointParams(1:(index-1)));
                    end
                    %find index of orientation_in_parent_parameter for
                    %z-direction
                    if jointStructOpt.OrientationInParent(index,3)
                        indexCountParent=sum([...
                            jointStructOpt.LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)]);
                        %store index: parent on top, child on bottom
                        constrainRZ(1,count)=indexCountParent+...
                            numParamsBefore;
                    end
                    %find index of orientation_in_child_parameter for
                    %z-direction
                    if jointStructOpt.OrientationInChild(index,3)
                        indexCountChild=sum([jointStructOpt...
                            .LocationInParent(index,:)...
                            jointStructOpt.OrientationInParent(index,:)...
                            jointStructOpt.LocationInChild(index,:)...
                            jointStructOpt.OrientationInChild(index,:)]);
                        %store index: parent on top, child on bottom
                        constrainRZ(2,count)=indexCountChild+...
                            numParamsBefore;
                    end
                    count=count+1;
                end
                %indicate whether parameter is reflected by using a
                %negative sign
                if jointPair.isReflectedOrnFlag(3)
                    constrainRZ(:,2)=-constrainRZ(:,2);
                end
            end %end if-statement for z-direction
            
            %store the index pairs for the constrained values, in
            %optimization, the change in the second column indices will
            %either be the same as or the negative of the first
            if jj==1
                cnstrPairIndices=[constrainTX;constrainTY;constrainTZ;...
                    constrainRX;constrainRY;constrainRZ];
            else
                cnstrPairIndices=[cnstrPairIndices;...
                    constrainTX;constrainTY;constrainTZ;...
                    constrainRX;constrainRY;constrainRZ];
            end
        end %end jj for-loop
        
        cnstrPairIndices(sum(abs(cnstrPairIndices),2)==0,:)=[];
        
        %prepare function output arguments
        varargout={cnstrPairIndices};
        
        end %end buildSymConstraints
        %==================================================================
        %   buildSymConstraints
        %==================================================================
        
        %==================================================================
        %   readTrcFile
        %==================================================================
        function varargout=readTrcFile(varargin)
        %readTrcFile
        %
        %   readTrcFile() opens a .trc file and extracts marker names and
        %   locations for use in the main kinematic calibration routine.
        %   
        %   The input argument to the function is the name of the .trc
        %   file.
        %   
        %   The function returns the marker names (in cell array), a matrix
        %   containing marker locations, and a time vector.
        %   
        %   Example function call:
        %   [markerNames,markerLocations,time]=readTrcFile(markerFileName);
        
        %populate variables
        expDataFilename=varargin{1};%.trc input filename
        
        %open .trc file
        fid=fopen(expDataFilename,'r');
        
        %skip first couple of lines
        nl=fgetl(fid);nl=fgetl(fid);nl=fgetl(fid);
        
        %extract desired variables
        values=sscanf(nl, '%f %f %f %f');%scan values
        numframes=values(3);%number of frames
        numcolumns=values(4)*3+2;%number of data columns
        numOfMarkers=values(4);%number of markers
        
        %again, skip un-wanted lines
        nl=fgetl(fid);nl=fgetl(fid);nl=fgetl(fid);
        
        %read data
        data=fscanf(fid,'%f',[numcolumns,numframes])';
        %close file
        fclose(fid);
        
        %convert data from mm to m
        expMarkerLocations=[data(:,3:end)]./1000;
        
        %time vector
        time=data(:,2);
        
        %open file again
        fid=fopen(expDataFilename,'r');
        
        %read all strings (cell array)
        txt=textscan(fid,'%s');
        
        %close file
        fclose(fid);
        
        %find appropriate indices to get marker names
        for itxt=1:length(txt{1})
            if strcmp(txt{1}(itxt),'Time')
                indxS=itxt;
                break;
            end
        end
        for jtxt=1:length(txt{1})
            if strcmp(txt{1}(jtxt),'X1')
                indxE=jtxt;
                break;
            end
        end
        
        %get marker names
        markerNames=txt{1}(indxS+1:indxE-1);
        
        %prepare function output arguments
        if (nargout==2)
            varargout={markerNames,expMarkerLocations};
        else
            varargout={markerNames,expMarkerLocations,time};
        end
        
        end %end readTrcFile
        %==================================================================
        %   readTrcFile
        %==================================================================
        
        %==================================================================
        %   rescaleModel
        %==================================================================
        function rescaleModel(varargin)
        %rescaleModel
        %
        %   rescaleModel() rescales the optimized model for cosmetic
        %   purposes.
        %   
        %   The input argument to this function is a structure array
        %   containing the following variables:
        %   v: design parameters
        %   dv: maximum allowable change
        %   vJoint: joint design parameters
        %   model: .osim model
        %   numOfJoints: number of joints
        %   allJointNames: joint names
        %   jointStructOpt: optimization specific joint structure
        %   optGroup: optimization group structure
        %   numOfJointParams: number of joint parameters
        %   optSettings: optimization settings
        %   markerPlateStructOpt: marker plate structure
        %   numOfModelJoints: number of model joints
        %   
        %   The function returns a scaled model.
        %   
        %   Example function call
        %   rescaleModel(inputArgStruct)
        
        %import OpenSim Matlab API functions
        import org.opensim.modeling.*
        
        %get input argument
        inputArgStruct=varargin{1};%input argument structure array
        
        %unpack variables from inputArgStruct
        v=inputArgStruct.v;
        dv=inputArgStruct.dv;
        vJoint=inputArgStruct.vJoint;
        model=inputArgStruct.model;
        numOfJoints=inputArgStruct.numOfJoints;
        allJointNames=inputArgStruct.allJointNames;
        jointStructOpt=inputArgStruct.jointStructOpt;
        optGroup=inputArgStruct.optGroup;
        numOfJointParams=inputArgStruct.numOfJointParams;
        optSettings=inputArgStruct.optSettings;
        markerPlateStructOpt=inputArgStruct.markerPlateStructOpt;
        numOfModelJoints=inputArgStruct.numOfModelJoints;
        
        dvJoint=dv(1:numel(vJoint));
        
        %find joint names and which parent body they are located within
        joint=cell(1,numOfJoints);
        parentBody=cell(1,numOfJoints);
        parentBodyName=parentBody;
        for jj=1:numOfJoints
            joint{jj}=model.getJointSet.get(jointStructOpt.jointName{jj});
            parentBodyName{jj}=char(joint{jj}.getParentName);
        end
        
        %find number of bodies to be scaled
        uniqueBodies=unique(parentBodyName);
        numOfUniqueBodies=numel(uniqueBodies);
        
        for jj=1:numOfUniqueBodies
            %get original locations of joints
            origLocsAll=cell(1,numOfModelJoints);
            
            %record location of all joints in parent bodies; this will be
            %rewritten at the end of scaling to ensure that no joint
            %offsets are altered by model scaling to avoid marker errors
            for kk=1:numOfModelJoints
                origLocs=Vec3(0);
                model.getJointSet.get(kk-1)...
                    .getLocationInParent(origLocs);
                origLocsAll{kk}=origLocs;
            end
            
            %determine which joints are in parent body
            whichJointsInBody=strcmp(uniqueBodies(jj),parentBodyName)';
            
            %get body which is being scaled
            body=model.getBodySet.get(uniqueBodies{jj});
            
            %find initial scale factors for body
            initScaleFactors=Vec3(0);
            body.getScaleFactors(initScaleFactors);
            initScaleFactors=char(initScaleFactors);
            initScaleFactors=str2num(initScaleFactors(2:end));
            newScaleFactors=ones(1,3);%new scale factors
            
            %scale in x-direction
            if (sum(jointStructOpt...
                    .LocationInParent(whichJointsInBody,1))>0 &&...
                    optGroup.scaleDirections(1))
                %determine which joints change in this direction
                whichJointsChange=whichJointsInBody.*jointStructOpt...
                    .LocationInParent(:,1);
                count=0;
                vEachJointX=zeros(sum(whichJointsChange),1);
                dvEachJointX=zeros(sum(whichJointsChange),1);
                %find v and dv parameters associated with these joints
                for kk=1:numOfJoints
                    if whichJointsChange(kk)
                        %find index where these joint parameters will be
                        %stored
                        if kk==1
                            numParamsBefore=0;
                        else
                            numParamsBefore=sum...
                                (numOfJointParams(1:(kk-1)));
                        end
                        %store parameters for this direction
                        count=count+1;
                        vEachJointX(count)=vJoint(numParamsBefore+1);
                        dvEachJointX(count)=dvJoint(numParamsBefore+1);
                    end
                end
                %find scale factor in this direction
                %check for excessive large scale factors or for NaN or inf
                %scale factors
                ScaleFactorX=((initScaleFactors(1).*abs...
                    (vEachJointX+dvEachJointX))./abs(vEachJointX))...
                    ./initScaleFactors(1);
                if logical(max(ScaleFactorX>2)) ||...
                        logical(max(ScaleFactorX<0.5))...
                        || logical(max(isnan(ScaleFactorX)))...
                        || logical(max(isinf(ScaleFactorX)))
                    msg1='\nWARNING: Invalid scale factor. Setting scale';
                    msg2=' to 1 in X direction for ';
                    fprintf(1,[msg1 msg2 char(body.getName) '!\n'])
                    ScaleFactorX(logical(isnan(ScaleFactorX)+...
                        isinf(ScaleFactorX)+(ScaleFactorX>2)+...
                        (ScaleFactorX<.5)))=1;
                end
                %find mean of scale factors for scaling multiple joints
                newScaleFactors(1)=mean(ScaleFactorX);
            end %end x-direction if-statement
            
            %scale in y-direction
            if (sum(jointStructOpt...
                    .LocationInParent(whichJointsInBody,2))>0 &&...
                    optGroup.scaleDirections(2))
                %determine which joints change in this direction
                whichJointsChange=whichJointsInBody.*jointStructOpt...
                    .LocationInParent(:,2);
                count=0;
                vEachJointY=zeros(sum(whichJointsChange),1);
                dvEachJointY=zeros(sum(whichJointsChange),1);
                %find v and dv parameters associated with these joints
                for kk=1:numOfJoints
                    if whichJointsChange(kk)
                        %find index where these joint parameters will be
                        %stored
                        if kk==1
                            numParamsBefore=jointStructOpt...
                                .LocationInParent(kk,1);
                        else
                            numParamsBefore=sum...
                                ([numOfJointParams(1:(kk-1))...
                                jointStructOpt.LocationInParent(kk,1)]);
                        end
                        %store parameters for this direction
                        count=count+1;
                        vEachJointY(count)=vJoint(numParamsBefore+1);
                        dvEachJointY(count)=dvJoint(numParamsBefore+1);
                    end
                end
                %find scale factor in this direction
                %check for excessive large scale factors or for NaN or inf
                %scale factors
                ScaleFactorY=((initScaleFactors(2).*abs...
                    (vEachJointY+dvEachJointY))./abs(vEachJointY))...
                    ./initScaleFactors(2);
                if logical(max(ScaleFactorY>2)) ||...
                        logical(max(ScaleFactorY<0.5)) ||...
                        logical(max(isnan(ScaleFactorY))) ||...
                        logical(max(isinf(ScaleFactorY)))
                    msg1='\nWARNING: Invalid scale factor. Setting scale';
                    msg2=' to 1 in Y direction for ';
                    fprintf(1,[msg1 msg2 char(body.getName) '!\n'])
                    ScaleFactorY(logical(isnan(ScaleFactorY)+...
                        isinf(ScaleFactorY)+(ScaleFactorY>2)+...
                        (ScaleFactorY<.5)))=1;
                end
                %find mean of scale factors for scaling multiple joints
                newScaleFactors(2)=mean(ScaleFactorY);
            end %end y-direction if-statement
            
            %scale in z-direction
            if (sum(jointStructOpt...
                    .LocationInParent(whichJointsInBody,3))>0 &&...
                    optGroup.scaleDirections(3))
                %determine which joints change in this direction
                whichJointsChange=whichJointsInBody.*jointStructOpt...
                    .LocationInParent(:,3);
                count=0;
                vEachJointZ=zeros(sum(whichJointsChange),1);
                dvEachJointZ=zeros(sum(whichJointsChange),1);
                %find v and dv parameters associated with these joints
                for kk=1:numOfJoints
                    if whichJointsChange(kk)
                        %find index where these joint parameters will be
                        %stored
                        if kk == 1
                            numParamsBefore=sum(jointStructOpt...
                                .LocationInParent(kk,1:2));
                        else
                            numParamsBefore=sum...
                                ([numOfJointParams(1:(kk-1))...
                                jointStructOpt.LocationInParent(kk,1:2)]);
                        end
                        %store parameters for this direction
                        count=count+1;
                        vEachJointZ(count)=vJoint(numParamsBefore+1);
                        dvEachJointZ(count)=dvJoint(numParamsBefore+1);
                    end
                end
                %find scale factor in this direction
                %check for excessive large scale factors or for NaN or inf
                %scale factors
                ScaleFactorZ=((initScaleFactors(3).*abs...
                    (vEachJointZ+dvEachJointZ))./abs(vEachJointZ))...
                    ./initScaleFactors(3);
                if logical(max(ScaleFactorZ>2)) ||...
                        logical(max(ScaleFactorZ<0.5)) ||...
                        logical(max(isnan(ScaleFactorZ))) ||...
                        logical(max(isinf(ScaleFactorZ)))
                    msg1='\nWARNING: Invalid scale factor. Setting scale';
                    msg2=' to 1 in Z direction for ';
                    fprintf(1,[msg1 msg2 char(body.getName) '!\n'])
                    ScaleFactorZ(logical(isnan(ScaleFactorZ)+...
                        isinf(ScaleFactorZ)+(ScaleFactorZ>2)+...
                        (ScaleFactorZ<.5)))=1;
                end
                %find mean of scale factors for scaling multiple joints
                newScaleFactors(3)=mean(ScaleFactorZ);
            end %end z-direction if-statement
            
            %scale model
            scaleSetObject=ScaleSet(optSettings.scaleSetFile);
            newScaleFactors=ArrayDouble.createVec3(newScaleFactors);
            scaleSetObject.get(uniqueBodies{jj})...
                .setScaleFactors(newScaleFactors);
            state=model.initSystem;
            model.scale(state,scaleSetObject);%scale
            
            %reset all joint locations since scaling
            %will sometimes shift the joints in unwanted manner
            for kk=1:numOfModelJoints
                model.getJointSet.get(kk-1)...
                    .set_location_in_parent(origLocsAll{kk});
                model.getJointSet.get(kk-1).upd_location_in_parent;
            end
            
            %reset the optimized joint locations in the body since scaling
            %will move them in undesired way
            utilFunctions.setParams(jointStructOpt,markerPlateStructOpt,...
                model,v,dv);
        end %end jj for-loop
        
        end %end rescaleModel
        %==================================================================
        %   rescaleModel
        %==================================================================
        
    end %end methods
end %end classdef