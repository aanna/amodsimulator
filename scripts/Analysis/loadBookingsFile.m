function [ bookings ] = loadBookingsFile( filename )
%LOADBOOKINGSFILE load the bookings file

fid = fopen(filename, 'r');
format_spec = '%d %f %d %f %f %d';
i=1;
bookings = {};
tline = fgetl(fid);
while ischar(tline)
	b = textscan(tline, format_spec);
    bookings(i,:) = b;
    tline = fgetl(fid);
    i = i + 1;
end

bookings = sortrows(bookings, 1); % sorted by id