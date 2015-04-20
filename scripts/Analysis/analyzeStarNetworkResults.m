%% parameters
doclear = false;
if doclear
    clear();
end

reload = true;

EVENT_DISPATCH = 0;
EVENT_MOVE= 1;
EVENT_ARRIVAL= 2;
EVENT_PICKUP= 3;
EVENT_DROPOFF= 4;
EVENT_LOCATION_VEHS_SIZE_CHANGE= 5;
EVENT_LOCATION_CUSTS_SIZE_CHANGE= 6;
EVENT_BOOKING_CANNOT_BE_SERVICED= 7;
EVENT_TELEPORT= 8;
EVENT_TELEPORT_ARRIVAL= 9;

filenames = {'../../spLog.txt', '../../maLog.txt', '../../mrLog.txt'};
titles = {'Simple Manager', 'Match Manager', 'Match Rebalance Manager'};
plot_colors = {'r', 'g', 'b'};
%filenames = {'../../mrLog.txt'};
%titles = {'Match Rebalance Manager'};
%% load the dataset
if reload
    events_to_skip = [EVENT_MOVE];
    events = {};
    nmgrs = length(filenames);
    for mid = 1:nmgrs
        events{mid} = loadEventsFile(filenames{mid}, events_to_skip);
    end
    bookings = loadBookingsFile('../../data/starnetwork_books.txt');
    nbookings = size(bookings,1);
end

%% match bookings, custid, booking time, dispatches, pickups, dropoff
BOOKING_ID_COL = 1;
BOOKING_TIME_COL = 2;
CUST_ID_COL = 3;
DISPATCH_COL = 4;
PICKUP_COL = 5;
DROPOFF_COL = 6;
amod_bookings = cell2mat(bookings(:,6)) > 0;

%%
results = {};
for mid=1:nmgrs
    fprintf('=== Loading %s ===\n', titles{mid});
    results{mid} = zeros(nbookings, 6);
    for j=1:3
        results{mid}(amod_bookings,j) = cell2mat(bookings(amod_bookings,j));
    end
    nevents{mid} = length(events{mid});
    for i=1:nevents{mid}
        e = events{mid}{i};
        switch e.type
            case EVENT_DISPATCH
                if length(e.entities) == 2
                    bid = e.entities(2);
                    if bid > 0
                        results{mid}(bid, DISPATCH_COL) = e.t;
                    end
                end
            case EVENT_PICKUP
                bid = e.entities(3);
                results{mid}(bid, PICKUP_COL) = e.t;
            case EVENT_DROPOFF
                bid = e.entities(3);
                results{mid}(bid, DROPOFF_COL) = e.t;
        end
    end
    
    % remove zero lines (Teleports)
    results{mid}( ~any(results{mid},2), : ) = [];  %rows
end    
%% plots

figure();
for mid = 1:nmgrs
    title(titles{mid});
    fprintf('=== %s ===\n', titles{mid});
    
    waiting_time = results{mid}(:, PICKUP_COL) - results{mid}(:, BOOKING_TIME_COL);
    %size(waiting_time)
    mean_waiting_time = mean(waiting_time);
    std_waiting_time = std(waiting_time);
    fprintf('Waiting time: %f (s.d. %f)\n', mean_waiting_time, std_waiting_time);
    fprintf('Min Max Waiting time: %f %f \n', min(waiting_time), max(waiting_time));
    subplot(2,1,1); hold on;
    [hx, binx] = hist(waiting_time, 50);
    plot(binx, hx, plot_colors{mid});
    title('Waiting time');
    
    travel_time = results{mid}(:, DROPOFF_COL) - results{mid}(:, PICKUP_COL);
    mean_travel_time = mean(travel_time);
    std_travel_time = std(travel_time);
    fprintf('Travel time: %f (s.d. %f)\n', mean_travel_time, std_travel_time);
    subplot(2,1,2); hold on;
    [hx, binx] = hist(travel_time, 50);
    plot(binx, hx, plot_colors{mid});
    title('Travel time');
end

legend(titles);
 