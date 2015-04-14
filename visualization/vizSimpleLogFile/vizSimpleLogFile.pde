import java.util.Map;
boolean ismac = false;
boolean issimmob = false;

// Event structure to load the data we load from the file
class Event {
  public int id;
  public int type;
  public float t;
  public float x;
  public float y;
  public float s;

  public Event(int id, int type, float t, float x, float y, float s) {
    this.id = id;
    this.type = type;
    this.t = t;
    this.x = x;
    this.y = y;
    this.s = s;
  }
  
  public Event() {
        this.id = 0;
    this.type = 0;
    this.t = 0;
    this.x = 0;
    this.y = 0;
    this.s = 0;
  }
}

float current_time = 0; // start time is 0
float time_window = 1; // 1 second

int current_event = 0; // starting event = 0

float min_x = 0;
float min_y = 0;
float max_x = 10000;
float max_y = 10000;
float scale_x = 1;
float scale_y = 1;
float range_x = 10000;
float range_y = 10000;

BufferedReader reader;

class Location {
  public float x, y, s;
}
HashMap<Integer,Location> locs = new HashMap<Integer,Location>();
void setup() {
  frameRate(50);
  // load the data file

  String filename = "/home/haroldsoh/Development/simmobility/dev/Basic/shared/entities/amodController/AMODBase/logfile.txt";

  if (ismac) {
    filename = "/Users/haroldsoh/Development/simmobility/dev/Basic/shared/entities/amodController/AMODBase/logfile.txt";
  }
  if (issimmob) {
    filename = "/home/haroldsoh/Development/simmobility/dev/Basic/logfile.txt";
  
   min_x = 365000; //365558.56;
   max_x = 377000; //376789.19;
   min_y = 140000;//140278.73;
   max_y = 144000;//142433.66;
   range_x = max_x - min_x;
   range_y = max_y - min_y;
}
  reader = createReader(filename);

  float w_width = 900;
  float w_height = (range_y/range_x)*700;
  size((int) w_width, (int) w_height);
  stroke(255);
  background(0, 0, 0);
  //smooth();
  // compute transformation vector
  scale_x = w_width/(max_x - min_x);
  scale_y = w_height/(max_y - min_y);

} 

void readLogFile(float end_time, ArrayList events) {
  String line;

  while (true) {
    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }
    if (line == null) return;

    String[] cols = split(line, ' ');
    Event e = new Event();
    if (parseInt(cols[3]) < 4) { //based on event id in AMODBase
     // moves, dropoffs or pickups
      e.id = parseInt(cols[2]);
      e.type = parseInt(cols[3]);
      e.t = parseFloat(cols[0]);
      e.x = parseFloat(cols[7]);
      e.y = parseFloat(cols[8]);
      e.s = 1;
    } else {
      // location
      e.id = parseInt(cols[2]);
      e.type = parseInt(cols[3]);
      e.t = parseFloat(cols[0]);
      e.x = parseFloat(cols[8]);
      e.y = parseFloat(cols[9]);
      e.s = parseFloat(cols[7]);
      int locid;
      
      if (e.type == 5) {
        String[] entities = split(cols[6], ',');
        locid = parseInt(entities[0]);
        //println(locid);
        Location loc = new Location();
        loc.x = e.x;
        loc.y = e.y;
        loc.s = e.s;
        locs.put(locid, loc);
      }
    }
    events.add(e);
    //println(e.id);
    if (e.t > end_time) break;
  };
}

void draw() {
  //fill(0, 0, 0, 50);
  //noStroke();
  //rect(0, 0, width, height);
  fill(0,0,0);
  rect(0, 0, width, height);
  float sc_factor = 10; //30
  float loc_s_factor = 1.0; //1.0


  //line(150, 25, mouseX, mouseY);
  float start_time = current_time;
  float end_time = start_time + time_window;
  ArrayList<Event> events = new ArrayList<Event>();
  readLogFile(end_time, events);
  
  // apply transformation
  pushMatrix();
  scale( scale_x, scale_y);
  translate(-min_x, -min_y);

  // draw locations
  stroke(100,100,100);
  fill(100,100,100,100);
    for (Map.Entry me : locs.entrySet()) {
      //println(me.getKey());
    Location l = (Location) me.getValue();
    ellipse(l.x, l.y, l.s*loc_s_factor, l.s*loc_s_factor);
  }
      noStroke();
  for (int i=0; i<events.size (); i++) {
    Event e = events.get(i);
    // if the event is after the end_time, break

    // draw the event
    // enum EventType {EVENT_MOVE, EVENT_ARRIVAL, EVENT_PICKUP, EVENT_DROPOFF};
    if (e.type == 0) {
      fill(#00B0FF); 
      ellipse(e.x, e.y, 5*sc_factor, 5*sc_factor);  // move event
    } else if (e.type == 1) {
      fill(#FFAF00); 
      ellipse(e.x, e.y, 8*sc_factor, 8*sc_factor);
    } else if (e.type == 2) {
      fill(#00C138); 
      ellipse(e.x, e.y, 10*sc_factor, 10*sc_factor);
    } else if (e.type == 3) {
      fill(#FF00E6); 
      ellipse(e.x, e.y, 12*sc_factor, 12*sc_factor);
    }
    println("Test:", e.x, e.y);
  }
  
  current_time = end_time;

  popMatrix();
}

