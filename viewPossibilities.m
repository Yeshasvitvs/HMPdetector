function viewPossibilities(motion, numFiles)

folderPossibilities = 'Results\';
folderTrials = 'Validation\old_format\';
win_size = 349;
dataFilesPossibilities = zeros(1,numFiles);
dataFilesTrial = zeros(1,numFiles);

for i=1:1:numFiles
    % retrieve the data of the possibilities file
    fid = fopen(sprintf('%spossibilities_%s_%d.txt',folderPossibilities,motion,i),'r')
    dataFilesPossibilities(i) = fid
    Cpossibilities = fscanf(fid,'%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n',[8,inf]);
    Cpossibilities = Cpossibilities';
    Cpossibilities = cat(1, zeros(win_size,8), Cpossibilities);
    % retrieve the data of the original trial
    nameTrial = sprintf('%s_test (%d).txt',motion,i);
    dataFilesTrial(i) = fopen([folderTrials nameTrial],'r');
    dataTrial = fscanf(dataFilesTrial(i),'%d\t%d\t%d\t%d\t%d\t%d\n',[6,inf]);
    % mapping from [-32768..+32767] to [-19.6133..+19.6133]
	noisy_x = (dataTrial(1,:)/65535)*(39.2266);
	noisy_y = (dataTrial(2,:)/65535)*(39.2266);
	noisy_z = (dataTrial(3,:)/65535)*(39.2266);
    
    x = 1:1:size(Cpossibilities,1);
    numSamples = length(noisy_x);
    time = 1:1:numSamples;
    figure;
        % display the raw acceleration recorded in the trial
        subplot(4,1,1);
        plot(time,noisy_x,'-');
        title(['Raw acceleration data of file: ', nameTrial]);
        axis([0 numSamples -19.6133 +19.6133]);
        subplot(4,1,2);
        plot(time,noisy_y,'-');
        axis([0 numSamples -19.6133 +19.6133]);
        subplot(4,1,3);
        plot(time,noisy_z,'-');
        axis([0 numSamples -19.6133 +19.6133]);
        % display the possibility values computed by the classifier
        subplot(4,1,4);
        plot(x,Cpossibilities(:,1), 'r');
        hold on;
        plot(x,Cpossibilities(:,2), 'g');
        hold on;
        plot(x,Cpossibilities(:,3), 'b');
        hold on;
        plot(x,Cpossibilities(:,4), 'm');
        hold on;
        plot(x,Cpossibilities(:,5), 'y');
        hold on;
        plot(x,Cpossibilities(:,6), 'c');
        hold on;
        plot(x,Cpossibilities(:,7), 'k');
        hold on;
        plot(x,Cpossibilities(:,8), '--r');
        hold on;
        title('Possibility values');
        h = legend('climb','drink','eat','get up','pour','sit','stand','walk',8,'Location','BestOutside');
        set(h,'Interpreter','none');
        axis([0 numSamples 0 1]);
end

end
