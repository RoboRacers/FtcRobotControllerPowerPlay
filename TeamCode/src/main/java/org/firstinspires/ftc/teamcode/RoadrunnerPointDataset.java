package org.firstinspires.ftc.teamcode;


import java.io.*;
import java.util.*;
public class RoadrunnerPointDataset {
    /*
     * This contains all the points for the field elements for roadrunner
    */

    enum PointType {
        G, // Ground
        L, // Low
        M, // Medium
        H, // High
        BS, // Blue Stack
        RS, // Red sTack
        T, // Terminal
        SI, // Signal
        BSUB, // Blue Subsation
        RSUB, // Red Rubstation
        SP, // Starting Point
        PP0, // Parking Position for SP0
        PP1, // Parking Positionfor SP1
        PP2, // Parking Position for SP2
        PP3 // Parking Position for SP3
    }
    public static void main(String[] args) 
    {
        // Type, Number, X, Y
        // Ground Junction
        Object[][] Points = {
            {PointType.G, 0, 0, 0},
            {PointType.G, 1, 48, 0},
            {PointType.G, 2, 48, 48},
            {PointType.G, 3, 0, 48},
            {PointType.G, 4, -48, 48},
            {PointType.G, 5, -48, 0},
            {PointType.G, 6, -48, -48},
            {PointType.G, 7, 0, -48},
            {PointType.G, 8, 48, -48},

            // Low Junction
            {PointType.L, 0, 48, 24},
            {PointType.L, 1, 24, 48},
            {PointType.L, 2, -24, 48},
            {PointType.L, 3, -48, 24},
            {PointType.L, 4, -48, -24},
            {PointType.L, 5, -24, -48},
            {PointType.L, 6, 24, -48},
            {PointType.L, 7, 48, -24},

            // Medium Junction
            {PointType.M, 0, 24, 24},
            {PointType.M, 1, -24, 24},
            {PointType.M, 2, -24, -24},
            {PointType.M, 3, 24, -24},
            
            // High Junction
            {PointType.H, 0, 24, 0},
            {PointType.H, 1, 0, 24},
            {PointType.H, 2, -24, 0},
            {PointType.H, 3, 0, -24},

            // Blue Stack
            {PointType.BS, 0, -12, 72},
            {PointType.BS, 1, -12, -72},

            // Red Stack
            {PointType.RS, 0, 12, 72},
            {PointType.RS, 1, -12, -72},

            // Terminal
            {PointType.T, 0, 60, 60},
            {PointType.T, 1, -60, 60},
            {PointType.T, 2, -60, -60},
            {PointType.T, 3, 60, -60},

            // Signal
            {PointType.SI, 0, 36, -36},
            {PointType.SI, 1, 36, 36},
            {PointType.SI, 2, -36, 36},
            {PointType.SI, 3, -36, -36},

            // Blue Substation
            {PointType.BSUB, 0, 0, -60},

            // Red SUbsation
            {PointType.RSUB, 0, 0, -60},

            // ONLY POINTS BELOW THIS LINE ARE UPDATED CORRECTLY
            // Starting Points
            // Subject to change based on the robot size
            {PointType.SP, 0, 36, -64.5},  // 64.5, 36 -> 36, -64.5
            {PointType.SP, 1, 36, 64.5},
            {PointType.SP, 2, -36, 64.5},
            {PointType.SP, 3, -36, -64.5},

            // Parking Postions for SP 0
            {PointType.PP0, 0, 12, -24},
            {PointType.PP0, 1, 36, -24},
            {PointType.PP0, 2, 60, -24},

            // Parking Postions for SP 1
            {PointType.PP1, 0, 60, 24},
            {PointType.PP1, 1, 36, 24},
            {PointType.PP1, 2, 12, 24},

            // Parking Postions for SP 2
            {PointType.PP2, 0, -12, 24},
            {PointType.PP2, 1, -36, 24},
            {PointType.PP2, 2, -60, 24},

            // Parking Postions for SP 3
            {PointType.PP3, 0, -60, -24},
            {PointType.PP3, 1, -36, -24},
            {PointType.PP3, 2, -12, -24}

        };
    }
    
}
