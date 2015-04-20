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
for mid=1:nmgrs
    results = zeros(nbookings, 6);
    for j=1:3
        results(amod_bookings,j) = cell2mat(bookings(amod_bookings,j));
    end
    nevents{mid} = length(events{mid});
    for i=1:nevents{mid}
        e = events{mid}{i};
        switch e.type
            case EVENT_DISPATCH
                if length(e.entities) == 2
                    bid = e.entities(2);
                    if bid > 0
                        results(bid, DISPATCH_COL) = e.t;
                    end
                end
            case EVENT_PICKUP
                bid = e.entities(3);
                results(bid, PICKUP_COL) = e.t;
            case EVENT_DROPOFF
                bid = e.entities(3);
                results(bid, DROPOFF_COL) = e.t;
        end
    end
    
    % remove zero lines (Teleports)
    results( ~any(results,2), : ) = [];  %rows
    
    %% plots
    figure();
    title(titles{mid});
    fprintf('=== %s ===\n', titles{mid});
    
    waiting_time = results(:, PICKUP_COL) - results(:, BOOKING_TIME_COL);
    mean_waiting_time = mean(waiting_time);
    std_waiting_time = std(waiting_time);
    fprintf('Waiting time: %f (s.d. %f)\n', mean_waiting_time, std_waiting_time);
    subplot(2,1,1);
    hist(waiting_time, 50);
    title('Waiting time');
    
    travel_time = results(:, DROPOFF_COL) - results(:, PICKUP_COL);
    mean_travel_time = mean(travel_time);
    std_travel_time = std(travel_time);
    fprintf('Travel time: %f (s.d. %f)\n', mean_travel_time, std_travel_time);
    subplot(2,1,2);
    hist(waiting_time, 50);
    title('Travel time');
end