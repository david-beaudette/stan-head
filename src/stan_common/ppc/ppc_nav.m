% Post processing script for Stan navigation
pkg load dataframe

[bagfilename, bagdir] = uigetfile('*.bag', ...
                                  'Select logged bag', ...
                                  '/home/david/ros1_ws/logs/stan_logs.bag');
                                  
% Convert bag topics to text
bagfilepath = fullfile(bagdir, bagfilename);
datadir = [bagdir 'extracted'];
topic_list = {'pitch_est', 'pitch_cmd', 'wireless/connection'};
datafile_list = cellfun(@(x) strrep(x, '/', '_'), topic_list, 'UniformOutput', false);
if ~exist(datadir, 'dir')
  mkdir(datadir);
end 
for topic_idx = 1:numel(topic_list)
  topic_cur = topic_list{topic_idx};
  datafile_cur = datafile_list{topic_idx};
  disp(['Processing topic ' topic_cur ' ...']);
  [sys_status, sys_out] = unix(['. /opt/ros/noetic/setup.sh ; ' ...
                                'rostopic echo -b ' bagfilepath ' -p /' topic_cur ...
                                ' > ' fullfile(datadir, datafile_cur) '.txt']);
  eval([datafile_cur ' =  dataframe ("' fullfile(datadir, datafile_cur) '.txt");']);
  disp(' done.');
end

%% Plot pitch
figure('Name', 'Pitch');