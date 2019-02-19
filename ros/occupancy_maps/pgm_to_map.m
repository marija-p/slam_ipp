image = imread('map_lab_trim.pgm');
%image_cropped = image(100:300, 50:350);
imshow(image)
image_bw = image < 100;
map = robotics.BinaryOccupancyGrid(image_bw, 20);