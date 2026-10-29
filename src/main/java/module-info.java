module drone.dronepathplanning_dijkstra {
    requires javafx.controls;
    requires javafx.fxml;


    opens drone.dronepathplanning_dijkstra to javafx.fxml;
    exports drone.dronepathplanning_dijkstra;
}