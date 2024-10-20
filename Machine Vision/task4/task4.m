clear all;
clc;
close all;


% Reading image
im = imread('Treasure_hard.jpg');
imshow(im);

% Binarisation
bin_threshold = 0.1;
bin_im = im2bw(im, bin_threshold);
imshow(bin_im);

% Extracting connected components
con_com = bwlabel(bin_im);
imshow(label2rgb(con_com));

% Computing objects properties
props = regionprops(con_com);

% Drawing bounding boxes
n_objects = numel(props);
imshow(im);
hold on;
for obj_idx = 1 : n_objects
    rectangle('Position', props(obj_idx).BoundingBox, 'EdgeColor', 'b');
end
hold off;

% Arrow/non-arrow determination
arrow_ind = arrow_finder(props, detect_yellow_comp(im));  

% Finding red arrow
start_arrow_id = detect_red_arrow(arrow_ind, props, im);

% Hunting
[cur_object, path, treasure_obj_Indices] = treasure_hunt(start_arrow_id, props, detect_yellow_comp(im), arrow_ind, im);

% Visualisation of the path
imshow(im);
hold on;
for path_idx = 1 : numel(path)
    obj_idx = path(path_idx);
    rectangle('Position', props(obj_idx).BoundingBox, 'EdgeColor', 'y');
    str = num2str(path_idx);
    text(props(obj_idx).BoundingBox(1), props(obj_idx).BoundingBox(2), str, 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 14);
end

% Visualisation of the treasure
for i = treasure_obj_Indices
    rectangle('Position', props(i).BoundingBox, 'EdgeColor', 'g');
end

% Function for extracting yellow dots component
%%
function yel_comp = detect_yellow_comp(im)
    yel_comp = regionprops(bwconncomp((im(:,:,1) > 150) & (im(:,:,2) > 200) & (im(:,:,3) < 100)), 'Area', 'Centroid', 'BoundingBox');
end

% Function to find the arrow ID
function [arrow_finder] = arrow_finder(props, detect_yellow_comp)
    nprops = length(props);
    n_yel_props = length(detect_yellow_comp);
    arrow_finder = [];
    for r = 1:nprops
        for s = 1:n_yel_props
            if(inside_box(detect_yellow_comp(s).Centroid,props(r).BoundingBox))
                arrow_finder(1,end+1) = r;
                arrow_finder(2,end) = s;
                break;
            end
        end        
    end
end



% Function to check the objects within the box
function inside_box = inside_box(k,l)
    if(k(1) > l(1) && k(1)<(l(1)+l(3)) && k(2)>l(2) && k(2)<(l(2)+l(4)))
       inside_box= true;
    else
        inside_box= false;
    end
end

% Function to find the red arrow
function start_arrow_id = detect_red_arrow(arrow_ind, props, im)
    n_arrows = numel(arrow_ind);
    start_arrow_id = 0;
    
    for arrow_num = 1 : n_arrows
        obj_idx = arrow_ind(arrow_num);
        centroid_colour = get_centroid_colour(obj_idx, props, im);
        
        if is_red(centroid_colour)
            start_arrow_id = obj_idx;
            break;
        end
    end
end

function centroid_colour = get_centroid_colour(obj_idx, props, im)
    centroid_colour = im(round(props(obj_idx).Centroid(2)), round(props(obj_idx).Centroid(1)), :);
end

function is_red = is_red(centroid_colour)
    is_red = centroid_colour(:, :, 1) > 240 && centroid_colour(:, :, 2) < 10 && centroid_colour(:, :, 3) < 10;
end



function [next_object, last_obj_status] = next_object_finder(cur_object, path, props, detect_yellow_comp, im, arrow_ind, treasure_obj_Indices)
    next_object = [];
    last_obj_status = 0;
    
    object_centroid = props(cur_object).Centroid;
    yellow_id = find(arrow_ind(1, :) == cur_object);
    yellow_centroid = detect_yellow_comp(arrow_ind(2, yellow_id)).Centroid;
    vector = yellow_centroid - object_centroid;
    direction = vector / max(abs(vector));
    
    object_list = setdiff(1:numel(props), [treasure_obj_Indices path]);
    if isempty(object_list)
        last_obj_status = 1;
        return;
    end

    direction_vector = object_centroid + vector + direction;
    while is_within_image_bounds(direction_vector, size(im)) && isempty(next_object)
        obj_indices = find(arrayfun(@(i) inside_box(direction_vector, props(i).BoundingBox), object_list));
        if ~isempty(obj_indices)
            next_object = object_list(obj_indices(1));
        end
        direction_vector = direction_vector + direction;
    end

    if isempty(next_object)
        last_obj_status = 1;
    end
end

function is_within_bounds = is_within_image_bounds(vector, image_size)
    is_within_bounds = abs(vector(1)) <= image_size(2) && abs(vector(2)) <= image_size(1);
end


function [cur_object, path, treasure_obj_Indices] = treasure_hunt(start_arrow_id, props, detect_yellow_comp, arrow_ind, im)
    cur_object = start_arrow_id;
    path = [];
    treasure_obj_Indices = [];
    last_obj_status = 0;
    
    while(last_obj_status==0)
        [cur_object, path] = process_arrow_objects(cur_object, arrow_ind, path, props, detect_yellow_comp, im, treasure_obj_Indices);
        [cur_object, last_obj_status, treasure_obj_Indices] = process_treasure_objects(cur_object, path, props, detect_yellow_comp, im, arrow_ind, treasure_obj_Indices);
    end
end

function [cur_object, path] = process_arrow_objects(cur_object, arrow_ind, path, props, detect_yellow_comp, im, treasure_obj_Indices)
    last_obj_status = 0;
    while ismember(cur_object, arrow_ind(1,:)) && last_obj_status == 0
        path(end + 1) = cur_object;
        [cur_object, last_obj_status] = next_object_finder(cur_object,path,props,detect_yellow_comp,im,arrow_ind,treasure_obj_Indices);  
    end
end

function [cur_object, last_obj_status, treasure_obj_Indices] = process_treasure_objects(cur_object, path, props, detect_yellow_comp, im, arrow_ind, treasure_obj_Indices)
    last_obj_status = 0;
    if last_obj_status == 0
        treasure_obj_Indices(end + 1) = cur_object;
        [cur_object, last_obj_status] = next_object_finder(path(end),path,props,detect_yellow_comp,im,arrow_ind,treasure_obj_Indices);
    end
end