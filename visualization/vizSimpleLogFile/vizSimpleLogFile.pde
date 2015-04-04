
// Event structure to load the data we load from the file
class Event {
  public int id;
  public int type;
  public float t;
  public float x;
  public float y;

  public Event(int id, int type, float t, float x, float y) {
    this.id = id;
    this.type = type;
    this.t = t;
    this.x = x;
    this.y = y;
  }
}

float current_time = 0; // start time is 0
float time_window = 1; // 1 second
ArrayList<Event> events = new ArrayList<Event>();
int current_event = 0; // starting event = 0

float min_x = -1;
float min_y = -1;
float max_x = -1;
float max_y = -1;
float scale_x = 1;
float scale_y = 1;

void setup() {
  frameRate(25);
  // load the data file
  String filename = "/Users/haroldsoh/Development/simmobility/dev/Basic/shared/entities/amodController/AMODBase/logfile.txt";
  loadLogFile(filename);
  
  float w_width = 500;
  float w_height = 500;
  size((int) w_width, (int) w_height);
  stroke(255);
  background(0, 0, 0);
  
  // compute transformation vector
  scale_x = w_width/(max_x - min_x);
  scale_y = w_height/(max_y - min_y);
  
  
} 

void loadLogFile(String filename) {
  String [] lines = loadStrings(filename);

  for (String l : lines) {
    String[] cols = split(l, ' ');
//    for (int i=0; i<cols.length; i++) {
//      println(cols[i]);
//    }
    // create event
    Event e = new Event( parseInt(cols[2]), parseInt(cols[3]), parseFloat(cols[0]), parseFloat(cols[7]), parseFloat(cols[8]));
    events.add(e);
    println(e.id, " ", e.t);
    min_x = min(e.x, min_x);
    max_x = max(e.x, max_x);
    min_y = min(e.y, min_y);
    max_y = max(e.y, max_y);
  }
}

void draw() {
  clear();
  pushMatrix();
  scale(scale_x, scale_y);
  
  //line(150, 25, mouseX, mouseY);
  float start_time = current_time;
  float end_time = start_time + time_window;
  
  int i = current_event;
  while (i < events.size()) {
    Event e = events.get(i);
    // if the event is after the end_time, break
    if (e.t > end_time) {
      current_event = i;
      break;
    }
    // draw the event
    // enum EventType {EVENT_MOVE, EVENT_ARRIVAL, EVENT_PICKUP, EVENT_DROPOFF};
    if (e.type == 0) {
      fill(#00B0FF); 
      ellipse(e.x, e.y, 5, 5);  // move event
    } else if (e.type == 1) {
      fill(#FFAF00); 
      ellipse(e.x, e.y, 8, 8);
    } else if (e.type == 2) {
      fill(#00C138); 
      ellipse(e.x, e.y, 10, 10);
    } else if (e.type == 3) {
      fill(#FF00E6); 
      ellipse(e.x, e.y, 12, 12);
    } 

    i++;
  }
  
  current_time = end_time;
  
  popMatrix();
}

