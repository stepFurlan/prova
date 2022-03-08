package provaDyn;

public class WorldTemperature {

	private double[][] tempMatrix;
	private int xWorldDimension;
	private int yWorldDimension;

	public WorldTemperature(int xWorldDimension, int yWorldDimension) {
		this.xWorldDimension = xWorldDimension;
		this.yWorldDimension = yWorldDimension;
		this.tempMatrix = new double[yWorldDimension][xWorldDimension];
		for (int i = 0; i < yWorldDimension; i++) {
			for (int j = 0; j < xWorldDimension; j++) {
				tempMatrix[i][j] = 0;
			}
		}
	}
	public double[][] getMatrix() {
		return this.tempMatrix;
	}

	public void setMatrixValue(int x, int y, double value) {
		this.tempMatrix[x][y] = value;
	}

	public double getMeanTemperature(double xx, double yy) {
		double result;
		int y = (int) Math.round(xx) + (int) (this.xWorldDimension + 12) / 2;
		int x = (int) Math.round(yy) + (int) (this.yWorldDimension + 12) / 2;
		double[][] temporaryMatrix = new double[yWorldDimension + 12][xWorldDimension + 12];
		for(int i = 6; i < yWorldDimension + 6; i++) {
			for(int j = 6; j < xWorldDimension + 6; j++) {
				temporaryMatrix[i][j] = tempMatrix[i-6][j-6];
			}
		}
		result = (temporaryMatrix[x][y] + temporaryMatrix[x][y + 1] + temporaryMatrix[x][y - 1] + temporaryMatrix[x - 1][y]
				+ temporaryMatrix[x + 1][y] + temporaryMatrix[x + 1][y + 1] + temporaryMatrix[x - 1][y + 1] + temporaryMatrix[x - 1][y - 1]
				+ temporaryMatrix[x + 1][y - 1] + temporaryMatrix[x - 2][y] + temporaryMatrix[x + 2][y] + temporaryMatrix[x + 2][y + 1]
				+ temporaryMatrix[x - 2][y + 1] + temporaryMatrix[x - 2][y - 1] + temporaryMatrix[x + 2][y - 1] + temporaryMatrix[x][y + 2]
				+ temporaryMatrix[x + 1][y + 2] + temporaryMatrix[x + 2][y + 2] + temporaryMatrix[x - 1][y + 2]
				+ temporaryMatrix[x - 2][y + 2] + temporaryMatrix[x][y - 2] + temporaryMatrix[x - 1][y - 2] + temporaryMatrix[x - 2][y - 2]
				+ temporaryMatrix[x + 1][y - 2] + temporaryMatrix[x + 2][y - 2] + temporaryMatrix[x - 3][y] + temporaryMatrix[x - 3][y + 1]
				+ temporaryMatrix[x - 3][y + 2] + temporaryMatrix[x - 3][y + 3] + temporaryMatrix[x - 3][y - 1]
				+ temporaryMatrix[x - 3][y - 2] + temporaryMatrix[x - 3][y - 3] + temporaryMatrix[x - 2][y - 3]
				+ temporaryMatrix[x - 1][y - 3] + temporaryMatrix[x][y - 3] + temporaryMatrix[x + 1][y - 3] + temporaryMatrix[x + 2][y - 3]
				+ temporaryMatrix[x + 3][y - 3] + temporaryMatrix[x + 3][y - 2] + temporaryMatrix[x + 3][y - 2]
				+ temporaryMatrix[x + 3][y - 1] + temporaryMatrix[x + 3][y] + temporaryMatrix[x + 3][y + 1] + temporaryMatrix[x + 3][y + 2]
				+ temporaryMatrix[x + 3][y + 3] + temporaryMatrix[x + 2][y + 3] + temporaryMatrix[x + 1][y + 3] + temporaryMatrix[x][y + 3]
				+ temporaryMatrix[x - 1][y + 3] + temporaryMatrix[x - 2][y + 3] + temporaryMatrix[x - 4][y - 4]
				+ temporaryMatrix[x - 3][y - 4] + temporaryMatrix[x - 2][y - 4] + temporaryMatrix[x - 1][y - 4] + temporaryMatrix[x][y - 4]
				+ temporaryMatrix[x + 1][y - 4] + temporaryMatrix[x + 2][y - 4] + temporaryMatrix[x + 3][y - 4]
				+ temporaryMatrix[x + 4][y - 4] + temporaryMatrix[x + 4][y - 3] + temporaryMatrix[x + 4][y - 2]
				+ temporaryMatrix[x + 4][y - 1] + temporaryMatrix[x + 4][y] + temporaryMatrix[x + 4][y + 1] + temporaryMatrix[x + 4][y + 2]
				+ temporaryMatrix[x + 4][y + 3] + temporaryMatrix[x + 4][y + 4] + temporaryMatrix[x + 3][y + 4]
				+ temporaryMatrix[x + 2][y + 4] + temporaryMatrix[x + 1][y + 4] + temporaryMatrix[x][y + 4] + temporaryMatrix[x - 1][y + 4]
				+ temporaryMatrix[x - 2][y + 4] + temporaryMatrix[x - 3][y + 4] + temporaryMatrix[x - 4][y + 4]
				+ temporaryMatrix[x - 4][y + 3] + temporaryMatrix[x - 4][y + 2] + temporaryMatrix[x - 4][y + 1] + temporaryMatrix[x - 4][y]
				+ temporaryMatrix[x - 4][y - 1] + temporaryMatrix[x - 4][y - 2] + temporaryMatrix[x - 4][y - 3]
				+ temporaryMatrix[x - 5][y - 5] + temporaryMatrix[x - 5][y - 4] + temporaryMatrix[x - 5][y - 3]
				+ temporaryMatrix[x - 5][y - 2] + temporaryMatrix[x - 5][y - 1] + temporaryMatrix[x - 5][y] + temporaryMatrix[x - 5][y + 1]
				+ temporaryMatrix[x - 5][y + 2] + temporaryMatrix[x - 5][y + 3] + temporaryMatrix[x - 5][y + 4]
				+ temporaryMatrix[x - 5][y + 5] + temporaryMatrix[x - 4][y + 5] + temporaryMatrix[x - 3][y + 5]
				+ temporaryMatrix[x - 2][y + 5] + temporaryMatrix[x - 1][y + 5] + temporaryMatrix[x][y + 5] + temporaryMatrix[x + 1][y + 5]
				+ temporaryMatrix[x + 2][y + 5] + temporaryMatrix[x + 3][y + 5] + temporaryMatrix[x + 4][y + 5]
				+ temporaryMatrix[x + 5][y + 5] + temporaryMatrix[x + 5][y + 4] + temporaryMatrix[x + 5][y + 3]
				+ temporaryMatrix[x + 5][y + 2] + temporaryMatrix[x + 5][y + 1] + temporaryMatrix[x + 5][y] + temporaryMatrix[x + 5][y - 1]
				+ temporaryMatrix[x + 5][y - 2] + temporaryMatrix[x + 5][y - 3] + temporaryMatrix[x + 5][y - 4]
				+ temporaryMatrix[x + 5][y - 5] + temporaryMatrix[x + 4][y - 5] + temporaryMatrix[x + 3][y - 5]
				+ temporaryMatrix[x + 2][y - 5] + temporaryMatrix[x + 1][y - 5] + temporaryMatrix[x][y - 5] + temporaryMatrix[x - 1][y - 5]
				+ temporaryMatrix[x - 2][y - 5] + temporaryMatrix[x - 3][y - 5] + temporaryMatrix[x - 4][y - 5]
				+ temporaryMatrix[x - 6][y - 6] + temporaryMatrix[x - 6][y - 5] + temporaryMatrix[x - 6][y - 4]
				+ temporaryMatrix[x - 6][y - 3] + temporaryMatrix[x - 6][y - 2] + temporaryMatrix[x - 6][y - 1] + temporaryMatrix[x - 6][y]
				+ temporaryMatrix[x - 6][y + 1] + temporaryMatrix[x - 6][y + 2] + temporaryMatrix[x - 6][y + 3]
				+ temporaryMatrix[x - 6][y + 4] + temporaryMatrix[x - 6][y + 5] + temporaryMatrix[x - 6][y + 6]
				+ temporaryMatrix[x - 5][y + 6] + temporaryMatrix[x - 4][y + 6] + temporaryMatrix[x - 3][y + 6]
				+ temporaryMatrix[x - 2][y + 6] + temporaryMatrix[x - 1][y + 6] + temporaryMatrix[x][y + 6] + temporaryMatrix[x + 1][y + 6]
				+ temporaryMatrix[x + 2][y + 6] + temporaryMatrix[x + 3][y + 6] + temporaryMatrix[x + 4][y + 6]
				+ temporaryMatrix[x + 5][y + 6] + temporaryMatrix[x + 6][y + 6] + temporaryMatrix[x + 6][y + 5]
				+ temporaryMatrix[x + 6][y + 4] + temporaryMatrix[x + 6][y + 3] + temporaryMatrix[x + 6][y + 2]
				+ temporaryMatrix[x + 6][y + 1] + temporaryMatrix[x + 6][y] + temporaryMatrix[x + 6][y - 1] + temporaryMatrix[x + 6][y - 2]
				+ temporaryMatrix[x + 6][y - 3] + temporaryMatrix[x + 6][y - 4] + temporaryMatrix[x + 6][y - 5]
				+ temporaryMatrix[x + 6][y - 6] + temporaryMatrix[x + 5][y - 6] + temporaryMatrix[x + 4][y - 6]
				+ temporaryMatrix[x + 3][y - 6] + temporaryMatrix[x + 2][y - 6] + temporaryMatrix[x + 1][y - 6] + temporaryMatrix[x][y - 6]
				+ temporaryMatrix[x - 1][y - 6] + temporaryMatrix[x - 2][y - 6] + temporaryMatrix[x - 3][y - 6]
				+ temporaryMatrix[x - 4][y - 6] + temporaryMatrix[x - 5][y - 6])/144;

		return result;
	}

}
