%% assumes data is in variable all_demands
% all_demands should be [time(since start of simulation), start_pos
% end_pos] where pos = [x,y]
% bin_width is the bin width (default is 3600 i.e. 1hr)
% stations 

bin_width = 3600;

kSecsInDay = 24*60*60;
%% create histograms 
% get the day for each row
day = floor(all_demands(:,1)/kSecsInDay) + 1;
secs = mod(all_demands(:,1), kSecsInDay);
max_day = max(day);

% find closest station to all positions
stid = knnsearch(stations, all_demands(:,2:3));

% generate time indexes
bins = (0:bin_width:(kSecsInDay-bin_width)) + bin_width/2;
nbins = length(bins);
hs = {};
mhs = [];
shs = [];
for s=1:nstations
    for d=1:max_day
       hs{s}(d,:) = hist(secs((day == d) & (stid == s)), bins);
    end
    
    mhs(s,:) = mean(hs{s});
    shs(s,:) = std(hs{s});
end
%% 
all_demands_hist = [(1:nstations)' mhs; (1:nstations)' shs];

%% generate training dataset
train_data = [];
for s=1:nstations
    for d=1:max_day-1
        train_data = [train_data; s*ones(nbins,1) (1:nbins)' hs{s}(d,:)'];
    end
end

for s=1:nstations
    for d=max_day
        test_data = [train_data; s*ones(nbins,1) (1:nbins)' hs{s}(d,:)'];
    end
end

%% GP Learning here
likfunc = @likGauss; sn = 0.1; hyp2.lik = log(sn);
%likfunc = @likT; sn = 10; nu= 1;  hyp2.lik = log([sn nu]);
covfunc = @covSEiso; hyp2.cov = [0; 0]; 
inffunc = @infExact
hyp2 = minimize(hyp2, @gp, -100, inffunc, [], covfunc, likfunc, train_data(:,1:2), train_data(:,3));
exp(hyp2.lik)
nlml2 = gp(hyp2, inffunc, [], covfunc, likfunc, train_data(:,1:2), train_data(:,3))

[m s2] = gp(hyp2, inffunc, [], covfunc, likfunc, train_data(:,1:2), train_data(:,3), test_data(:,1:2), test_data(:,3));

RMSE = sqrt(mean((m - test_data(:,3)).^2))
plot(m, test_data(:,3))
%%
pred_mhs = [];
pred_shs = [];
for i=1:size(test_data,1)
    pred_mhs(test_data(i,1), test_data(i,2)) = m(i);
    pred_shs(test_data(i,1), test_data(i,2)) = sqrt(s2(i));
end

all_pred_demands_hist = [pred_mhs; pred_shs];
%% output demand histograms (mean and stdev)
dlmwrite('../../data/starnetwork_all_demands_hist.txt', [nstations bin_width], 'delimiter', ' ');
dlmwrite('../../data/starnetwork_all_demands_hist.txt', all_demands_hist, '-append', 'delimiter', ' ');


%% output predicted GP demand histograms (mean and stdev)
dlmwrite('../../data/starnetwork_all_pred_demands_hist.txt', [nstations bin_width], 'delimiter', ' ');
dlmwrite('../../data/starnetwork_all_pred_demands_hist.txt', all_pred_demands_hist, '-append', 'delimiter', ' ');