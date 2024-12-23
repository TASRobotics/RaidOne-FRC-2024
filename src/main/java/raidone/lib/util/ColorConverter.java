package raidone.lib.util;

public class ColorConverter {

    public static int[] hueToRGB(float hue) {
                // Convert hue from 0-255 range to 0-360 range
                float hueInDegrees = (hue / 255.0f) * 360.0f;
        
                float C = 1.0f; // Full saturation and brightness (chroma)
                float X = C * (1 - Math.abs((hueInDegrees / 60.0f) % 2 - 1));
                float m = 0.0f; // Lightness correction factor (since we're assuming full brightness)
        
                float r = 0, g = 0, b = 0;
        
                if (0 <= hueInDegrees && hueInDegrees < 60) {
                    r = C;
                    g = X;
                    b = 0;
                } else if (60 <= hueInDegrees && hueInDegrees < 120) {
                    r = X;
                    g = C;
                    b = 0;
                } else if (120 <= hueInDegrees && hueInDegrees < 180) {
                    r = 0;
                    g = C;
                    b = X;
                } else if (180 <= hueInDegrees && hueInDegrees < 240) {
                    r = 0;
                    g = X;
                    b = C;
                } else if (240 <= hueInDegrees && hueInDegrees < 300) {
                    r = X;
                    g = 0;
                    b = C;
                } else if (300 <= hueInDegrees && hueInDegrees < 360) {
                    r = C;
                    g = 0;
                    b = X;
                }
        
                // Convert the RGB values to the range of 0 to 255
                int red = Math.round((r + m) * 255);
                int green = Math.round((g + m) * 255);
                int blue = Math.round((b + m) * 255);
        
                return new int[]{red, green, blue};
    }

}
