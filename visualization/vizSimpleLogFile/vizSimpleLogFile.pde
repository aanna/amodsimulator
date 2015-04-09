
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

int current_event = 0; // starting event = 0

float min_x = 0;
float min_y = 0;
float max_x = 10000;
float max_y = 10000;
float scale_x = 1;
float scale_y = 1;

boolean ismac = false;

BufferedReader reader;

void setup() {
  frameRate(20);
  // load the data file

  String filename = "/home/haroldsoh/Development/simmobility/dev/Basic/shared/entities/amodController/AMODBase/logfile.txt";
  
  if (ismac) {
    filename = "/Users/haroldsoh/Development/simmobility/dev/Basic/shared/entities/amodController/AMODBase/logfile.txt";
  }
  reader = createReader(filename);

  float w_width = 500;
  float w_height = 500;
  size((int) w_width, (int) w_height);
  stroke(255);
  background(0, 0, 0);
  smooth();
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
    Event e = new Event( parseInt(cols[2]), parseInt(cols[3]), parseFloat(cols[0]), parseFloat(cols[7]), parseFloat(cols[8]));
    events.add(e);
    println(e.id, " ", e.t);
    if (e.t > end_time) break;
  };
}

void draw() {
  fill(0, 0, 0, 50);
  noStroke();
  rect(0, 0, width, height);
  float sc_factor = 30;
  pushMatrix();
  scale(scale_x, scale_y);

  //line(150, 25, mouseX, mouseY);
  float start_time = current_time;
  float end_time = start_time + time_window;
  ArrayList<Event> events = new ArrayList<Event>();
  readLogFile(end_time, events);
  println(events.size());
  for (int i=0; i<events.size(); i++) {
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
  }

  current_time = end_time;

  popMatrix();
}

