//Copyright (c) 2016-2021 Diego Settimi - https://github.com/arkypita/

// This program is free software; you can redistribute it and/or modify  it under the terms of the GPLv3 General Public License as published by  the Free Software Foundation; either version 3 of the License, or (at  your option) any later version.
// This program is distributed in the hope that it will be useful, but  WITHOUT ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv3  General Public License for more details.
// You should have received a copy of the GPLv3 General Public License  along with this program; if not, write to the Free Software  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,  USA. using System;

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;

namespace LaserGRBL
{
	// Island raster engraving: burnable pixels are split into islands separated by white space,
	// where every island has a single contiguous span on each scan line (shapes that branch, like a ring,
	// are cut into several islands). Each island is engraved serpentine on its own extent only,
	// and islands are visited in nearest-neighbour order to minimize travel.
	public partial class GrblFile
	{
		private void ImageIslands2Line(Bitmap bmp, L2LConf c)
		{
			bool uni = Settings.GetObject("Unidirectional Engraving", false);

			IslandRasterizer raster = new IslandRasterizer(bmp, c, uni);
			IslandGCodeWriter writer = new IslandGCodeWriter(c, raster.Axis, skipcmd);

			foreach (IslandStep step in raster.Order())
				raster.Engrave(step, writer);

			list.AddRange(OptimizeLine2Line(writer.Commands, c));
		}

		private class RasterIsland
		{
			public int VMin;                    //first scan line covered by the island cells
			public int[] UMin;                  //first burning pixel of each line (indexed by v - VMin)
			public int[] UMax;                  //last burning pixel of each line, -1 if the line is empty
			public int FirstLine = int.MaxValue;
			public int LastLine = -1;
			public int LineCount;               //number of non-empty lines
			public bool Visited;

			public bool HasLine(int v) { return UMax[v - VMin] >= 0; }
			public int Min(int v) { return UMin[v - VMin]; }
			public int Max(int v) { return UMax[v - VMin]; }
		}

		private struct IslandEntry
		{
			public bool Descending;             //start from LastLine and move towards FirstLine
			public bool Reverse;                //first line is engraved towards decreasing u

			public IslandEntry(bool descending, bool reverse)
			{
				Descending = descending;
				Reverse = reverse;
			}
		}

		private struct IslandStep
		{
			public RasterIsland Island;
			public IslandEntry Entry;
		}

		private enum IslandAxis { Horizontal, Vertical, Diagonal }

		//pixel boundary t along scan line v, to image coordinates (pixels)
		//diagonal lines follow x + y = v from top-left to bottom-right, like the legacy diagonal mode
		private static void ToXY(IslandAxis axis, int t, int v, out int x, out int y)
		{
			if (axis == IslandAxis.Horizontal) { x = t; y = v; }
			else if (axis == IslandAxis.Vertical) { x = v; y = t; }
			else { x = t; y = v + 1 - t; }
		}

		// u = coordinate along the scan line, v = scan line index
		private class IslandRasterizer
		{
			public readonly IslandAxis Axis;
			private readonly bool mPwm;
			private readonly bool mUni;
			private readonly int mW;
			private readonly int mH;
			private readonly int mU;
			private readonly int mV;
			private readonly int mRadius;     //gap bridging radius (pixels)
			private readonly byte[] mDark;      //darkness of each pixel (0 = white)
			private readonly int[] mPower = new int[256];
			private readonly bool[] mBurn = new bool[256];
			private readonly List<RasterIsland> mIslands = new List<RasterIsland>();
			private readonly IslandEntry[] mEntries;

			public IslandRasterizer(Bitmap bmp, L2LConf c, bool uni)
			{
				if (c.dir == RasterConverter.ImageProcessor.Direction.Vertical) Axis = IslandAxis.Vertical;
				else if (c.dir == RasterConverter.ImageProcessor.Direction.Diagonal) Axis = IslandAxis.Diagonal;
				else Axis = IslandAxis.Horizontal;
				mPwm = c.pwm;
				mUni = uni;
				mW = bmp.Width;
				mH = bmp.Height;
				mU = Axis == IslandAxis.Vertical ? mH : mW;
				mV = Axis == IslandAxis.Horizontal ? mH : Axis == IslandAxis.Vertical ? mW : mW + mH - 1;

				for (int d = 0; d < 256; d++)
				{
					mPower[d] = d == 0 ? 0 : c.pwm ? d * (c.maxPower - c.minPower) / 255 + c.minPower : d;
					mBurn[d] = c.pwm ? mPower[d] > 0 : mPower[d] > 125;
				}

				mEntries = uni
					? new IslandEntry[] { new IslandEntry(false, false), new IslandEntry(true, false) }
					: new IslandEntry[] { new IslandEntry(false, false), new IslandEntry(false, true), new IslandEntry(true, false), new IslandEntry(true, true) };

				double res = c.vectorfilling ? c.fres : c.res;
				mRadius = Math.Max(0, (int)Math.Round(c.islandGap * res / 2.0));

				mDark = ReadDarkness(bmp);
				BuildIslands(BuildBridgedMask());
			}

			private int PixelIndex(int u, int v)
			{
				if (Axis == IslandAxis.Horizontal) return v * mW + u;
				if (Axis == IslandAxis.Vertical) return u * mW + v;
				return (v - u) * mW + u;
			}

			private bool Burns(int u, int v)
			{
				if (Axis == IslandAxis.Diagonal && (v - u < 0 || v - u >= mH))
					return false; //outside the image
				return mBurn[mDark[PixelIndex(u, v)]];
			}

			//burnable pixels dilated by mRadius in both directions (indexed v * mU + u): white gaps up to ~islandGap get bridged
			private byte[] BuildBridgedMask()
			{
				int r = mRadius;
				byte[] rows = new byte[mU * mV];
				int[] prefix = new int[mU + 1];
				for (int v = 0; v < mV; v++)
				{
					for (int u = 0; u < mU; u++)
						prefix[u + 1] = prefix[u] + (Burns(u, v) ? 1 : 0);
					if (prefix[mU] == 0)
						continue;
					for (int u = 0; u < mU; u++)
						if (prefix[Math.Min(mU, u + r + 1)] - prefix[Math.Max(0, u - r)] > 0)
							rows[v * mU + u] = 1;
				}

				byte[] mask = new byte[mU * mV];
				int[] count = new int[mU];
				for (int v = 0; v < Math.Min(r, mV); v++)
					AddRow(rows, count, v, 1);
				for (int v = 0; v < mV; v++)
				{
					if (v + r < mV) AddRow(rows, count, v + r, 1);
					if (v - r - 1 >= 0) AddRow(rows, count, v - r - 1, -1);
					for (int u = 0; u < mU; u++)
						if (count[u] > 0)
							mask[v * mU + u] = 1;
				}
				return mask;
			}

			private void AddRow(byte[] rows, int[] count, int v, int sign)
			{
				int o = v * mU;
				for (int u = 0; u < mU; u++)
					count[u] += rows[o + u] * sign;
			}

			//split the bridged mask into strips that have a single contiguous span on every line:
			//each span continues the strip of the most overlapping span on the previous line, otherwise it starts a new strip
			private void BuildIslands(byte[] mask)
			{
				List<int> stripStart = new List<int>();
				List<List<int>> stripA = new List<List<int>>();
				List<List<int>> stripB = new List<List<int>>();

				List<int> prevA = new List<int>(), prevB = new List<int>(), prevStrip = new List<int>();
				List<int> curA = new List<int>(), curB = new List<int>(), curStrip = new List<int>();
				for (int v = 0; v < mV; v++)
				{
					curA.Clear(); curB.Clear(); curStrip.Clear();
					int o = v * mU;
					for (int u = 0; u < mU; u++)
					{
						if (mask[o + u] == 0)
							continue;
						int a = u;
						while (u + 1 < mU && mask[o + u + 1] != 0)
							u++;
						curA.Add(a);
						curB.Add(u);
					}

					bool[] prevUsed = new bool[prevA.Count];
					int p0 = 0;
					for (int i = 0; i < curA.Count; i++)
					{
						int a = curA[i], b = curB[i];
						while (p0 < prevA.Count && prevB[p0] < a)
							p0++;

						int best = -1, bestOverlap = 0;
						for (int p = p0; p < prevA.Count && prevA[p] <= b; p++)
						{
							int overlap = Math.Min(b, prevB[p]) - Math.Max(a, prevA[p]) + 1;
							if (!prevUsed[p] && overlap > bestOverlap)
							{
								best = p;
								bestOverlap = overlap;
							}
						}

						int strip;
						if (best >= 0)
						{
							prevUsed[best] = true;
							strip = prevStrip[best];
						}
						else
						{
							strip = stripStart.Count;
							stripStart.Add(v);
							stripA.Add(new List<int>());
							stripB.Add(new List<int>());
						}
						stripA[strip].Add(a);
						stripB[strip].Add(b);
						curStrip.Add(strip);
					}

					List<int> t;
					t = prevA; prevA = curA; curA = t;
					t = prevB; prevB = curB; curB = t;
					t = prevStrip; prevStrip = curStrip; curStrip = t;
				}

				//the engraved extent of each line is limited to the real burnable pixels inside the strip span
				for (int s = 0; s < stripStart.Count; s++)
				{
					RasterIsland isl = new RasterIsland();
					isl.VMin = stripStart[s];
					int lines = stripA[s].Count;
					isl.UMin = new int[lines];
					isl.UMax = new int[lines];
					for (int j = 0; j < lines; j++)
					{
						int v = isl.VMin + j;
						isl.UMin[j] = int.MaxValue;
						isl.UMax[j] = -1;
						for (int u = stripA[s][j]; u <= stripB[s][j]; u++)
						{
							if (!Burns(u, v))
								continue;
							if (u < isl.UMin[j]) isl.UMin[j] = u;
							isl.UMax[j] = u;
						}

						if (isl.UMax[j] >= 0)
						{
							isl.FirstLine = Math.Min(isl.FirstLine, v);
							isl.LastLine = Math.Max(isl.LastLine, v);
							isl.LineCount++;
						}
					}

					if (isl.LineCount > 0)
						mIslands.Add(isl);
				}
			}

			private static byte[] ReadDarkness(Bitmap bmp)
			{
				int w = bmp.Width, h = bmp.Height;
				byte[] rv = new byte[w * h];
				BitmapData data = bmp.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
				try
				{
					byte[] row = new byte[w * 4];
					for (int y = 0; y < h; y++)
					{
						Marshal.Copy(new IntPtr(data.Scan0.ToInt64() + (long)y * data.Stride), row, 0, row.Length);
						for (int x = 0; x < w; x++)
							rv[y * w + x] = (byte)((255 - row[x * 4 + 2]) * row[x * 4 + 3] / 255); //BGRA, same formula as GetColor
					}
				}
				finally
				{
					bmp.UnlockBits(data);
				}
				return rv;
			}

			private void EntryPoint(RasterIsland isl, IslandEntry e, out int x, out int y)
			{
				int v = e.Descending ? isl.LastLine : isl.FirstLine;
				int u = e.Reverse ? isl.Max(v) + 1 : isl.Min(v);
				ToXY(Axis, u, v, out x, out y);
			}

			private void ExitPoint(RasterIsland isl, IslandEntry e, out int x, out int y)
			{
				int v = e.Descending ? isl.FirstLine : isl.LastLine;
				bool lastReverse = !mUni && (e.Reverse ^ ((isl.LineCount - 1) % 2 == 1));
				int u = lastReverse ? isl.Min(v) : isl.Max(v) + 1;
				ToXY(Axis, u, v, out x, out y);
			}

			//greedy nearest neighbour on the island entry corners, using a bucket grid to keep it fast with many islands
			public List<IslandStep> Order()
			{
				List<IslandStep> path = new List<IslandStep>();
				int n = mIslands.Count;
				if (n == 0)
					return path;

				int bs = Math.Max(1, (int)Math.Ceiling(Math.Max(mW, mH) / Math.Sqrt(n)));
				int gu = mW / bs + 2;
				int gv = mH / bs + 2;
				List<int>[] grid = new List<int>[gu * gv];

				for (int i = 0; i < n; i++)
				{
					foreach (IslandEntry e in mEntries)
					{
						int x, y;
						EntryPoint(mIslands[i], e, out x, out y);
						int b = (y / bs) * gu + (x / bs);
						if (grid[b] == null)
							grid[b] = new List<int>();
						if (grid[b].Count == 0 || grid[b][grid[b].Count - 1] != i)
							grid[b].Add(i);
					}
				}

				int curX = 0, curY = 0;
				int maxR = Math.Max(gu, gv);
				for (int done = 0; done < n; done++)
				{
					int bcu = Math.Min(gu - 1, Math.Max(0, curX / bs));
					int bcv = Math.Min(gv - 1, Math.Max(0, curY / bs));
					long bestD = long.MaxValue;
					IslandStep best = new IslandStep();

					for (int r = 0; r <= maxR; r++)
					{
						for (int gy = bcv - r; gy <= bcv + r; gy++)
						{
							if (gy < 0 || gy >= gv)
								continue;

							bool fullRow = r == 0 || gy == bcv - r || gy == bcv + r;
							for (int gx = bcu - r; gx <= bcu + r; gx += fullRow ? 1 : 2 * r)
							{
								if (gx < 0 || gx >= gu)
									continue;

								List<int> bucket = grid[gy * gu + gx];
								if (bucket == null)
									continue;

								for (int j = bucket.Count - 1; j >= 0; j--)
								{
									RasterIsland isl = mIslands[bucket[j]];
									if (isl.Visited)
									{
										bucket[j] = bucket[bucket.Count - 1];
										bucket.RemoveAt(bucket.Count - 1);
										continue;
									}

									foreach (IslandEntry e in mEntries)
									{
										int x, y;
										EntryPoint(isl, e, out x, out y);
										long dx = x - curX, dy = y - curY;
										long d = dx * dx + dy * dy;
										if (d < bestD)
										{
											bestD = d;
											best.Island = isl;
											best.Entry = e;
										}
									}
								}
							}
						}

						//every bucket outside ring r is at least r * bs away from the current position
						long ringDist = (long)r * bs;
						if (best.Island != null && bestD <= ringDist * ringDist)
							break;
					}

					best.Island.Visited = true;
					path.Add(best);
					ExitPoint(best.Island, best.Entry, out curX, out curY);
				}

				return path;
			}

			public void Engrave(IslandStep step, IslandGCodeWriter writer)
			{
				RasterIsland isl = step.Island;
				bool reverse = step.Entry.Reverse;
				int first = step.Entry.Descending ? isl.LastLine : isl.FirstLine;
				int last = step.Entry.Descending ? isl.FirstLine : isl.LastLine;
				int dir = step.Entry.Descending ? -1 : 1;

				for (int v = first; v != last + dir; v += dir)
				{
					if (!isl.HasLine(v))
						continue;

					int a = isl.Min(v), b = isl.Max(v);
					writer.Travel(reverse ? b + 1 : a, v);

					int prevKey = -1;
					for (int i = 0; i <= b - a; i++)
					{
						int u = reverse ? b - i : a + i;
						int key = PixelKey(u, v);
						if (prevKey != -1 && key != prevKey)
							writer.Run(reverse ? u + 1 : u, prevKey);
						prevKey = key;
					}
					writer.Run(reverse ? a : b + 1, prevKey);

					if (!mUni)
						reverse = !reverse;
				}
			}

			//strip spans on a line never overlap, so every pixel inside a span belongs to that strip
			private int PixelKey(int u, int v)
			{
				int d = mDark[PixelIndex(u, v)];
				if (!mBurn[d])
					return 0;
				return mPwm ? mPower[d] : 1;
			}
		}

		private class IslandGCodeWriter
		{
			public readonly List<GrblCommand> Commands = new List<GrblCommand>();
			private readonly L2LConf mConf;
			private readonly IslandAxis mAxis;
			private readonly double mRes;
			private readonly string mSkipCmd;
			private bool mFast = true;
			private int mLine;

			public IslandGCodeWriter(L2LConf c, IslandAxis axis, string skipcmd)
			{
				mConf = c;
				mAxis = axis;
				mRes = c.vectorfilling ? c.fres : c.res;
				mSkipCmd = skipcmd;
			}

			public void Travel(int u, int v)
			{
				mLine = v;
				Add(true, Coords(u, true, true), OffPower());
			}

			//move along the current scan line up to u; key is the power (pwm) or 1 = laser on, 0 = laser off
			public void Run(int u, int key)
			{
				string coords = Coords(u, mAxis != IslandAxis.Vertical, mAxis != IslandAxis.Horizontal);
				if (key == 0)
					Add(true, coords, OffPower());
				else
					Add(false, coords, mConf.pwm ? FormatPower(key) : mConf.lOn);
			}

			private void Add(bool fast, string coords, string power)
			{
				string mode = fast != mFast ? (fast ? mSkipCmd : "G1") + " " : "";
				mFast = fast;
				Commands.Add(new GrblCommand(mode + coords + " " + power));
			}

			private string Coords(int u, bool withX, bool withY)
			{
				int x, y;
				ToXY(mAxis, u, mLine, out x, out y);
				if (withX && withY) return string.Format("X{0} Y{1}", Format(x, mConf.oX), Format(y, mConf.oY));
				return withX ? "X" + Format(x, mConf.oX) : "Y" + Format(y, mConf.oY);
			}

			private string OffPower()
			{ return mConf.pwm ? FormatPower(0) : mConf.lOff; }

			private string FormatPower(int power)
			{
				if (mConf.firmwareType == Firmware.Smoothie)
					return string.Format(System.Globalization.CultureInfo.InvariantCulture, "S{0:0.00}", power / 255.0);
				else
					return string.Format(System.Globalization.CultureInfo.InvariantCulture, "S{0}", power);
			}

			private string Format(int pixel, float offset)
			{ return Math.Round(pixel / mRes + offset, 3).ToString(System.Globalization.CultureInfo.InvariantCulture); }
		}
	}
}
