import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./drone.css";
import DroneApp from "./DroneApp";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <DroneApp />
  </StrictMode>,
);
