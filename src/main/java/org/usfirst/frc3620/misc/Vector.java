package org.usfirst.frc3620.misc;

/**
 * Add your docs here.
 */
public class Vector {
    @Override
		public String toString() {
			return "[d=" + direction + ", m=" + magnitude + "]";
		}

		public double getDirection() {
			return direction;
		}

		public double getMagnitude() {
			return magnitude;
		}
		
		private double direction;
		private double magnitude;
		
		public Vector(double _direction, double _magnitude) {
			direction = _direction;
			magnitude = _magnitude;
		}
	}