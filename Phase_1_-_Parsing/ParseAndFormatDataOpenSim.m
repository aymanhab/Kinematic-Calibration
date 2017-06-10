function ParseAndFormatDataOpenSim
% This function will load and parse all data in the directory that is in
% csv and trc files with the word "Gait" in them. Additionally, it will
% resample the GRF and marker data to 141 time points (20% of the cycle
% before/after the cycle begins/ends) and will reorder that data from
% Dr.Patten's lab into a format suitable for OpenSim.

% 1 means that these files will be written
WriteTRC = 1;
WriteGRF = 1;
WriteEMG = 1;
PlotParsedGRFs = 0;

close all

% Assign which side is being studies, the output data will go for two heel
% strikes of the specified leg
ParseSide = 'left';

% Assign new order for GRF and marker data
% for example to transform x y z to -y z x would be [-2 3 1]
% to maintain same coordinate system, use [1 2 3]
% SIMM coordinate system is defined as follows:
% x: anterior-posterior (forward-backward)
% y: superior-inferior (up-down)
% z: medial-lateral (side to side)
GRFReordering = [2 -3 -1]; % Tranforms force plate coordinates to SIMM coordinates
MarkerReordering = [-3 2 1]; % Transforms marker data coordinates to SIMM coordinates

% Force plate centers
FPCenter1 = [23.850000 81.250000 0.000000]*10;
FPCenter2 = [73.300000 81.250000 0.000000]*10;

% Distance from force plate center to electrical center where loads are
% being recorded
FPCtoEC1 = [-24.580000 -85.869995 1.820000]*10;
FPCtoEC2 = [23.850000 -85.869995 1.650000]*10;

EC1 = ones(141,1)*(FPCenter1+FPCtoEC1);
EC2 = ones(141,1)*(FPCenter2+FPCtoEC2);

% Confirmed in bertec force plate manual
% http://bertec.com/uploads/pdfs/manuals/Instrumented%20Treadmill%20Manual.pdf
EC1 = EC1(:,abs(GRFReordering));% Left force plate origin
EC2 = EC2(:,abs(GRFReordering));% Right force plate origin

EC1(:,1) = EC1(:,1)-13;
EC2(:,3) = EC2(:,3)-13; 

% Corners of the force plate for making plots
Corners1 = [0.000031	1625
    476.999969	1625
    476.999969	0
    0.000031	0
    0.000031	1625];%/1000;

% Corners of the force plate for making plots
Corners2 = [494.500031	1625
    971.5	1625
    971.5	0
    494.500031	0
    494.500031	1625];%/1000;

% Find all gait trc files
GaitFiles = dir('*Gait*.trc');

for i = 1:numel(GaitFiles)
    
    %% Load Data
    TRCFile = GaitFiles(i).name
    GRFFile = strrep(TRCFile,'.TRC','.csv');
    
    % load data from the TRC files and GRF files using custom algorithm
    [MarkerData, MarkerNames, SampleRateM, TimeM] = LoadTRCFile(TRCFile);
    [GRFData, GRFHeaders, SampleRateGRF, TimeGRF] = LoadGRFCSV(GRFFile);
    
    EMGData = GRFData(:,13:end);
    EMGnames = GRFHeaders(13:end);
    GRFData = GRFData(:,1:12);
    
    %% Filter Data

%     GRFDataParsing25_2 = filterData(GRFData, 1/SampleRateGRF, 25,2);
%     GRFDataParsing20_2 = filterData(GRFData, 1/SampleRateGRF, 20,2);
%     GRFDataParsing15_2 = filterData(GRFData, 1/SampleRateGRF, 15,2);
%     GRFDataParsing25_1 = filterData(GRFData, 1/SampleRateGRF, 25,1);
%     GRFDataParsing20_1 = filterData(GRFData, 1/SampleRateGRF, 20,1);
%     GRFDataParsing = filterData(GRFData, 1/SampleRateGRF, 10,2);
    GRFDataParsing = filterDataOld(GRFData, 1/SampleRateGRF, 10,2);

%     GRFDataParsing = woltring_filter_only(GRFData(1:2000,:), 3, 6, TimeGRF(1:2000,:), TimeGRF(1:2000,:));
%     keyboard
    
%     GRFDataParsing6_2 = filterData(GRFData, 1/SampleRateGRF, 6,2);
%     GRFData = GRFDataParsing;

%     Ratio = abs(GRFDataParsing(:,2)./GRFDataParsing(:,3));

%   Scale GRFs for ICA
%     GRFDataScaled = (GRFDataParsing-ones(length(GRFDataParsing),1)*(range(GRFDataParsing)/2+min(GRFDataParsing)))./(ones(length(GRFDataParsing),1)*range(GRFDataParsing));

% %     
%     [~,Smoothed] = spaps(TimeGRF,GRFData(:,3),1e4,2);

%     Fs = 1/mean(diff(TimeGRF));                    % Sampling frequency
%     T = TimeGRF(end);                                    % Sample time
%     L = length(GRFData(:,7));                     % Length of signal
%     t = TimeGRF;                % Time vector
%     y = GRFData(:,3);     % Sinusoids plus noise
%     
%     NFFT = 2^nextpow2(L); % Next power of 2 from length of y
%     Y = fft(y,NFFT)/L;
%     f = Fs/2*linspace(0,1,NFFT/2+1);
% 
%     % Plot single-sided amplitude spectrum.
%     plot(f,2*abs(Y(1:NFFT/2+1))) 
%     title('Single-Sided Amplitude Spectrum of y(t)')
%     xlabel('Frequency (Hz)')
%     ylabel('|Y(f)|')
    
    % Process EMG data HP Filter -> Demean -> Rectify -> LP Filter
%     EMGDataProcessed = processEMG(EMGData, 1/SampleRateGRF, 30, 4);
    
    %% Assign which side used for basis of parsing data
    
    % Find vertical ground reaction force from limb being used
    if strcmp(ParseSide,'right')
        GRFVert = GRFDataParsing(:,abs(GRFReordering(2))+6);
    else
        GRFVert = GRFDataParsing(:,abs(GRFReordering(2)));
    end
    
    % Find approximate location of heel strikes and toes offs (will be off by
    % 10-50 milliseconds), this is done by finding instances where the
    % vertical GRF either approaches or moves away from zero
    [Peaks, HeelStrikeFrames] = findpeaks(-abs(GRFVert+20),'MINPEAKHEIGHT',-2);
    
    % Remove toe off instances by determining whether GRFs are increasing
    % or decreasing
    Slope = diff(GRFVert);
    HeelStrikeFrames(Slope(HeelStrikeFrames)>0)=[];
    if HeelStrikeFrames(1) < 201
        HeelStrikeFrames(1) = [];
    end
    
    % Correct the heel strike locations so that they are much closer to the
    % actual heel strike time frame by finding the frame where the slope is
    % zero nearest to the previous guess
    numHeelStrikes = numel(HeelStrikeFrames);
    for j = 1:numHeelStrikes
        Window = (HeelStrikeFrames(j)-200):HeelStrikeFrames(j);
        [Peaks, FrameCorrection] = findpeaks(-abs(Slope(Window)),'MINPEAKHEIGHT',-.05);
        HeelStrikeFrames(j) = HeelStrikeFrames(j)-200+FrameCorrection(end);
    end
    
    %     Plot GRFs and heel strike times to check for accuracy of heel strike
    %     detection
    if PlotParsedGRFs
            plot(-GRFVert);
            hold on
            plot(HeelStrikeFrames,-GRFVert(HeelStrikeFrames), 'rx')
            pause(10)
    end
            
    % Determine the beginnings and ends of cycles
    CycleBegins = HeelStrikeFrames(1:(end-1));
    CycleEnds = HeelStrikeFrames(2:end);
    
    numCycleFrames = CycleEnds-CycleBegins;
    
    % Remove final heel strike if there is not enough time following the
    % it to pad the data effectively
    if (HeelStrikeFrames(end)+.2*numCycleFrames(end))>numel(GRFVert)
        HeelStrikeFrames(end) = [];
    end
    if (HeelStrikeFrames(1)-.2*numCycleFrames(1))<=0
        HeelStrikeFrames(1) = [];
    end
    numHeelStrikes = numel(HeelStrikeFrames);
    
    % Determine the beginnings and ends of cycles now that some cycles
    % might have been removed
    CycleBegins = HeelStrikeFrames(1:(end-1));
    CycleEnds = HeelStrikeFrames(2:end);
    
    % Add padding at the beginnings and ends of the cycles
    numCycleFrames = CycleEnds-CycleBegins;
    CycleBegins = CycleBegins-.2*numCycleFrames;
    CycleEnds = CycleEnds+.2*numCycleFrames;
    
    % Round time frames up to nearest factor of 10 to maintain consistency
    % between marker data and GRF data sampling rates
    CycleBegins = ceil((CycleBegins-1)/10)*10;
    CycleEnds = ceil((CycleEnds-1)/10)*10;
    
    % Now seperate the data into individual steps will result in only one
    % step for overground trials
    numSteps = numHeelStrikes-1;
    GRFDataParsed = cell(numSteps,1);
    MarkerDataParsed = cell(numSteps,1);
    EMGDataParsed = cell(numSteps,1);
    TimeAnalog = cell(numSteps,1);
    TimeMarker = cell(numSteps,1);
    for j = 1:numSteps
        
        AnalogDataFrames = CycleBegins(j):CycleEnds(j);
        MarkerDataFrames = ceil(AnalogDataFrames(1:10:end)/10)+1;
        
        TimeAnalog{j} = TimeGRF(AnalogDataFrames);
        TimeMarker{j} = TimeM(MarkerDataFrames);
        
        GRFDataParsed{j} = GRFData(AnalogDataFrames,:);
        MarkerDataParsed{j} = MarkerData(MarkerDataFrames,:);
        EMGDataParsed{j} = EMGData(AnalogDataFrames,:);
        
    end
    
    % Filter and Resample marker and GRF data to 141 time points
    TimeResampled = cell(numSteps,1);
    for j = 1:numSteps
        
        TimeAnalogStep = TimeAnalog{j};
        TimeMarkerStep = TimeMarker{j};
        
        Period = (TimeMarkerStep(end)-TimeMarkerStep(1))*(101/141);
        
        GRFDataParsed{j} = filterData(GRFDataParsed{j}, 1/SampleRateGRF, 7/Period,2);
%         GRFDataParsed{j} = filterData(GRFDataParsed{j}, 1/SampleRateGRF, 7/Period,1);
        MarkerDataParsed{j} = filterData(MarkerDataParsed{j}, 1/SampleRateM, 7/Period,3);
        
        TimeNew = linspace(TimeMarkerStep(1), TimeMarkerStep(end), 141);
        
        TimeResampled{j} = TimeNew(:);
        
        GRFDataParsed{j} = spline(TimeAnalogStep, GRFDataParsed{j}', TimeNew)';
        MarkerDataParsed{j} = spline(TimeMarkerStep, MarkerDataParsed{j}', TimeNew)';
%         EMGDataParsed{j} = spline(TimeAnalogStep, EMGDataParsed{j}', TimeNew)';
        
    end
    
    % Reorder Marker and GRF data according to protocol specified at the
    % top of the file
    for j = 1:numSteps
        GRFDataParsed{j} = reorderData(GRFDataParsed{j},GRFReordering);
        MarkerDataParsed{j} = reorderData(MarkerDataParsed{j}, MarkerReordering);
    end
    
    [~, HeelMarkerIndexR] = max(strcmp(MarkerNames,'R.Heel'));
    [~, HeelMarkerIndexL] = max(strcmp(MarkerNames,'L.Heel'));
    [~, HeelMarkerIndexR2] = max(strcmp(MarkerNames,'R.Toe'));
    [~, HeelMarkerIndexL2] = max(strcmp(MarkerNames,'L.Toe'));
    
    % Determine center of pressure values and compare with Vicon
    % calculations and also set all GRFs less than 10 N to zero, change
    % moments to N-m
    for j = 1:numSteps
        GRFDataStep = GRFDataParsed{j};
        
        % Remove offsets
        [minVal1] = min(GRFDataStep(:,2));
        cutoff1 = minVal1+30;
        indices1 = GRFDataStep(:,2)<cutoff1;
        Offset1 = mean(GRFDataStep(indices1,1:6));
        
        [minVal2] = min(GRFDataStep(:,8));
        cutoff2 = minVal2+30;
        indices2 = GRFDataStep(:,8)<cutoff2;
        Offset2 = mean(GRFDataStep(indices2,7:12));
        
%         GRFDataStepOld = GRFDataStep;
        GRFDataStep = GRFDataStep-ones(141,1)*[Offset1 Offset2];
        
        % Set low values of the GRF Data to zero
%         GRFDataStep(GRFDataStep(:,2)<10,1:6) = 0;
%         GRFDataStep(GRFDataStep(:,8)<10,7:12) = 0;
        
        % Compute COP
        COP1Xorig = (GRFDataStep(:,6)-1e-3*EC1(:,2).*GRFDataStep(:,1))./GRFDataStep(:,2)+EC1(:,1)/1000;
        COP1Zorig = -(GRFDataStep(:,4)+1e-3*EC1(:,2).*GRFDataStep(:,3))./GRFDataStep(:,2)+EC1(:,3)/1000;
        
        COP2Xorig = (GRFDataStep(:,12)-1e-3*EC2(:,2).*GRFDataStep(:,1))./GRFDataStep(:,8)+EC2(:,1)/1000;
        COP2Zorig = -(GRFDataStep(:,10)+1e-3*EC2(:,2).*GRFDataStep(:,3))./GRFDataStep(:,8)+EC2(:,3)/1000;
        
        % Convert moment into units of N-m
        GRFDataStep(:,[4:6 10:12]) = GRFDataStep(:,[4:6 10:12])/1000;
        
        % Append electrical center info on GRF data for input into open sim
        GRFDataStep = [GRFDataStep EC1/1000 EC2/1000];
        
        % Output the new GRFs
        GRFDataParsed{j} = GRFDataStep;
        
%         plot(Corners1(:,1),Corners1(:,2),'b-x')
%         hold on
%         plot(Corners2(:,1),Corners2(:,2),'b-x'  )
%         plot(EC1(1,3), EC1(1,1) ,'go')
%         plot(EC2(1,3), EC2(1,1),'ro')
%         
%         MarkerDataStep = MarkerDataParsed{j};
%         RightMarkerData = MarkerDataStep(:,(HeelMarkerIndexR-1)*3+(1:3));
%         LeftMarkerData = MarkerDataStep(:,(HeelMarkerIndexL-1)*3+(1:3));
%         
%         RightMarkerData2 = MarkerDataStep(:,(HeelMarkerIndexR2-1)*3+(1:3));
%         LeftMarkerData2 = MarkerDataStep(:,(HeelMarkerIndexL2-1)*3+(1:3));
%         
%         plot(COP1Zorig+EC1(1,3), COP1Xorig+EC1(1,1), 'gx')
%         plot(COP2Zorig+EC2(1,3), COP2Xorig+EC2(1,1), 'rx')
%         plot(RightMarkerData(:,3),RightMarkerData(:,1),'ro');
%         plot(LeftMarkerData(:,3),LeftMarkerData(:,1),'go');
%         plot(RightMarkerData2(:,3),RightMarkerData2(:,1),'ro');
%         plot(LeftMarkerData2(:,3),LeftMarkerData2(:,1),'go');
%         axis([-.05 1 -.1 1.7]*1000)
%         
        
    end
    
    %% Now write the data to files
    %parfor j = 1:numSteps
    for j = 1:numSteps

        
        if j < 10
            TrialNum = ['_0' num2str(j)];
        else
            TrialNum = ['_' num2str(j)];
        end
        
        outFileTRC = strrep(TRCFile,'.TRC',['_' ParseSide TrialNum '.trc']);
        outFileGRF = strrep(TRCFile,'.TRC',['_' ParseSide TrialNum '_GRF.mot']);
        outFileEMG = strrep(TRCFile,'.TRC',['_' ParseSide TrialNum '_EMG.mot']);
        
        if WriteTRC
            outputTRCFile(outFileTRC, MarkerDataParsed{j},TimeResampled{j},MarkerNames);
        end
        if WriteGRF
            outputGRFmot(outFileGRF, GRFDataParsed{j},TimeResampled{j});
        end
        if WriteEMG
            outputEMGmot(outFileEMG, EMGDataParsed{j},TimeAnalog{j},EMGnames);
        end
        
    end
    
end

function emgFiltered = processEMG(emgData, dt, HPcutoff, LPcutoff)
% Function that process EMG data according the the process described by
% Lloyd and Besier (2003)
% Inputs:
%       emgData is a M x N matrix of unfiltered EMG data where M is the
%           number of time frames and N is the number of muscles
%       dt is the time step used in the EMG data
%       HPcutoff is the cutoff frequency for the high pass filter
%       LPcutoff is the cutoff frequency for the low pass filter
% Outputs:
%       emgFiltered is the filtered EMG data

SampRate = 1/mean(dt);

% High pass filter the data
degree = 4;
[z,p,k] = butter(degree, HPcutoff/SampRate*2,'high');
[sos,g] = zp2sos(z,p,k);      % Convert to SOS(second-order-sections) form
EMG_HP = filtfilt(sos,g,emgData);

% Demean
EMG_DM = EMG_HP-ones(size(EMG_HP,1),1)*mean(EMG_HP);

% Rectify
EMG_RF = abs(EMG_DM);

% Low pass filter
degree = 4;
[z,p,k] = butter(degree,LPcutoff/SampRate*2);
[sos,g] = zp2sos(z,p,k);      % Convert to SOS(second-order-sections) form
% pad data with final values
numPad = 0;
EMG_Padded = [ones(numPad,1)*EMG_RF(1,:);EMG_RF;ones(numPad,1)*EMG_RF(end,:)];
EMG_LP = filtfilt(sos,g,EMG_Padded);
emgFiltered = EMG_LP(numPad+1:end-numPad,:);

% Remove any negative EMG values that may still exist
emgFiltered(emgFiltered<0) = 0;

function [MarkerData, MarkerNames, SampleRate, Time] = LoadTRCFile(inFile)

% Read 5 header lines from input .trc file
fid = fopen(inFile);
header1 = fgetl(fid);
header2 = fgetl(fid);
header3 = fgetl(fid);
header4 = fgetl(fid);
header5 = fgetl(fid);

% Read marker data as one long column
data = fscanf(fid, '%f');
fclose(fid);

% Reshape marker data based on number of rows and
% columns of marker data specified in the input data
% file, with two extra columns for frame and time
info = sscanf(header3,'%f %f %d %d');
nrows = info(3,1);
ncols = info(4,1)*3;
data = reshape(data, ncols+2, nrows)';

SampleRate = info(1,1);

MarkerNames = strrep(header4,'Patient 4:','');
MarkerNames = textscan(MarkerNames,'%s');
MarkerNames = MarkerNames{1};
MarkerNames(1:2) = [];

Time = data(:,2);
MarkerData = data(:,3:end);

function [Data, Header, SampleRate, Time] = LoadGRFCSV(inFile)

% Read 5 header lines from input .trc file
fid = fopen(inFile);
header1 = fgetl(fid);
header2 = fgetl(fid);
header3 = fgetl(fid);
header4 = fgetl(fid);
header5 = fgetl(fid);

% Reshape marker data based on number of rows and
% columns of marker data specified in the input data
% file, with two extra columns for frame and time
data = textscan(fid, '%f','Delimiter',',');
fclose(fid);

SampleRate = str2double(strrep(header2,',Hz',''));

Header = textscan(header3,'%s','Delimiter',',');
Header = Header{1};
numCols = numel(Header);
Header(1) = [];

data = data{1};
numRows = numel(data)/numCols;

data = reshape(data,numCols,numRows)';

Time = (data(:,1)-1)/SampleRate;
Data = data(:,2:end);

function Data = filterData(Data, dt, cutoff,degree)

if strcmp(cutoff,'VarFreq')
    cutoff = 7/(mean(dt)*101);
end

SampRate = 1/mean(dt);
[b,a] = butter(degree,cutoff/SampRate*2);
Data = filtfilt(b,a,Data);

function Data = filterDataOld(Data, dt, cutoff,degree)

if strcmp(cutoff,'VarFreq')
    cutoff = 7/(mean(dt)*101);
end

numTrials = size(Data, 2);
SampRate = 1/mean(dt);
[z,p,k] = butter(degree,cutoff/SampRate*2);
[sos,g] = zp2sos(z,p,k);      % Convert to SOS(second-order-sections) form
Hd = dfilt.df2tsos(sos,g);
% pad data with final values
numPad = 60;
MData_Padded = [ones(numPad,1)*Data(1,:);Data;ones(numPad,1)*Data(end,:)];
MData_LowPass_1_padded = zeros(size(MData_Padded,1),numTrials);
MData_LowPass_2_padded = zeros(size(MData_Padded,1),numTrials);
for j=1:numTrials
    MData_LowPass_1_padded(:,j) = filter(Hd,MData_Padded(:,j));
end
% Filter backwards to remove time delay/lag
for j=1:numTrials
    MData_LowPass_2_padded(end:-1:1,j) = filter(Hd,MData_LowPass_1_padded(end:-1:1,j));
end
Data = MData_LowPass_2_padded(numPad+1:end-numPad,:);

function Data = reorderData(DataOrig, NewOrder)
% Assign new order for GRF and marker data
% for example to transform x y z to -y z x would be [-2 3 1]
% to maintain same coordinate system, use [1 2 3]
% SIMM coordinate system is defined as follows:
% x: anterior-posterior (forward-backward)
% y: superior-inferior (up-down)
% z: medial-lateral (side to side)

Data = DataOrig;

newSign = ones(size(Data,1),1)*sign(NewOrder);
newOrder = abs(NewOrder);

numItems = size(Data,2)/3;
for i = 1:numItems
    Data(:,3*(i-1)+1:3*i) = DataOrig(:,newOrder+3*(i-1)).*newSign;
end

function outputGRFmot(outFile,GRFData,Time)

Data = [Time GRFData];

Header = [outFile sprintf('\n') ...
    'version=1' sprintf('\n') ...
    'nRows=' num2str(size(Data,1)) sprintf('\n') ...
    'nColumns=' num2str(size(Data,2)) sprintf('\n') ...
    'inDegrees=no' sprintf('\n') ...
    'endheader' sprintf('\n') ...
    'time' sprintf('\t') 'F1x' sprintf('\t') 'F1y' sprintf('\t') 'F1z' sprintf('\t') 'M1x' sprintf('\t')...
    'M1y' sprintf('\t') 'M1z' sprintf('\t') 'F2x' sprintf('\t') 'F2y' sprintf('\t') 'F2z' sprintf('\t')...
    'M2x' sprintf('\t') 'M2y' sprintf('\t') 'M2z' sprintf('\t') 'EC1x' sprintf('\t')...
    'EC1y' sprintf('\t') 'EC1z' sprintf('\t') 'EC2x' sprintf('\t')...
    'EC2y' sprintf('\t') 'EC2z' sprintf('\n')];

nRows = size(Data,1);
nColumns = size(Data,2);

DataString = '';
for j = 1:nRows
    for k = 1:nColumns
        if k ~= nColumns
            DataString = [DataString num2str(Data(j,k)) sprintf('\t')];
        else
            DataString = [DataString num2str(Data(j,k)) sprintf('\n')];
        end
    end
end

fid = fopen(outFile,'w');
fprintf(fid,'%s',[Header DataString]);
fclose(fid);

function outputEMGmot(outFile,EMGData,Time, EMGnames)

Data = [Time EMGData];

Header = [outFile sprintf('\n') ...
    'version=1' sprintf('\n') ...
    'nRows=' num2str(size(Data,1)) sprintf('\n') ...
    'nColumns=' num2str(size(Data,2)) sprintf('\n') ...
    'inDegrees=no' sprintf('\n') ...
    'endheader' sprintf('\n') ...
    'time' sprintf('\t')];

for i = 1:length(EMGnames)
    if i ~= length(EMGnames)
        Header = [Header EMGnames{i} sprintf('\t')];
    else
        Header = [Header EMGnames{i} sprintf('\n')];
    end
end

nRows = size(Data,1);
nColumns = size(Data,2);
DataString = '';
for j = 1:nRows
    for k = 1:nColumns
        if k ~= nColumns
            DataString = [DataString num2str(Data(j,k)) sprintf('\t')];
        else
            DataString = [DataString num2str(Data(j,k)) sprintf('\n')];
        end
    end
end

fid = fopen(outFile,'w');
fprintf(fid,'%s',[Header DataString]);
fclose(fid);

function outputTRCFile(outFile, MarkerData,TimeVector,markerNames)

nMarkers = length(markerNames);

Header = HeaderTRC;

rate = 1/mean(diff(TimeVector));
AllMarkersTRC = MarkerData;

MarkerLabels = markerNames;
Header = strrep(Header,'DATARATE_INS',sprintf('%.12f',rate));
Header = strrep(Header,'CAMRATE_INS',sprintf('%.12f',rate));
Header = strrep(Header,'ORIGRATE_INS',sprintf('%.12f',rate));
Header = strrep(Header,'NFRAMES_INS',sprintf('%d',length(TimeVector)));
Header = strrep(Header,'NMARKERS_INS',sprintf('%d',nMarkers));
Header = strrep(Header,'UNITS_INS','mm');
Header = strrep(Header,'FILENAME',outFile);

% Create Marker Labels String
RemoveXYZLabel = 0;
if length(MarkerLabels)==(nMarkers*3)
    MarkerLabels=MarkerLabels([1:3:end]);
    RemoveXYZLabel = 1;
end
MarkerLabelString ='';
XYZString ='';
for m=1:nMarkers
    if length(MarkerLabels)<m
        label = ['Marker' num2str(m)];
    else
        label = MarkerLabels{m};
        if RemoveXYZLabel
            label = strrep(label,'_x','');
        end
    end
    XYZString = [XYZString sprintf('X%d\tY%d\tZ%d\t',m,m,m)];
    MarkerLabelString = [MarkerLabelString sprintf('%s\t\t\t',label)];
end
Header = strrep(Header,'FIRSTMARKERNAME_INS',MarkerLabelString);
Header = strrep(Header,'XYZ_INS',XYZString);

%% Create Data string
DataString = '';
for f=1:length(TimeVector)
    DataString = [DataString sprintf('%d\t%.12f\t',f,TimeVector(f))];
    for m=1:nMarkers
        DataString = [DataString sprintf('%.12f\t%.12f\t%.12f\t',AllMarkersTRC(f,(m-1)*3+1),AllMarkersTRC(f,(m-1)*3+2),AllMarkersTRC(f,(m-1)*3+3))];
    end
    DataString = [DataString sprintf('\n')];
end
Header = strrep(Header,'DATASTART_INS',DataString);

fid = fopen(outFile,'w');
fprintf(fid,'%s',Header);
fclose(fid);

function header = HeaderTRC

header = ['PathFileType	4	(X/Y/Z)	FILENAME.trc' sprintf('\n') ...
    'DataRate	CameraRate	NumFrames	NumMarkers	Units	OrigDataRate	OrigDataStartFrame	OrigNumFrames' sprintf('\n') ...
    'DATARATE_INS	CAMRATE_INS	NFRAMES_INS	NMARKERS_INS	UNITS_INS	ORIGRATE_INS	1	       NFRAMES_INS' sprintf('\n') ...
    'Frame#	Time	FIRSTMARKERNAME_INS' sprintf('\n') ...
    sprintf('\t')  sprintf('\t')	'XYZ_INS' sprintf('\n') ...
    '' sprintf('\n') ...
    'DATASTART_INS' sprintf('\n')];