% numImages = 9;
% files = cell(1, numImages);
% for i = 1:numImages
%     files{i} = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
%         'calibration', 'slr', sprintf('image%d.jpg', i));
% end
% 
% % Display one of the calibration images
% magnification = 25;
% I = imread(files{1});
% figure; imshow(I, InitialMagnification = magnification);
% title("One of the Calibration Images");





counter = 1;
baseDir = pwd;
baseName = 'image_'; %image_ is the name i have chosen for the image files
newName = fullfile(baseDir, 'cam_clib', sprintf('%s%d.jpg', baseName, counter));

cam = webcam("USB Camera");

for a = 1:1
    img = snapshot(cam);
    while exist(newName,'file')
    counter = counter + 1;
    newName = fullfile(baseDir, 'cam_clib', sprintf('%s%d.jpg', baseName, counter));
    end
    imwrite(img, newName);
end









