SELECT person_id, prev_stop_departure_time, 
	prev_stop_location, stop_location, 
        stop_mode, arrival_time
  FROM day_activity_schedule_2012
  ORDER BY prev_stop_departure_time;
