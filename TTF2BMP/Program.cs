using System;
using System.IO;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using System.Drawing.Text;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using System.Linq;

namespace TTF2BMP
{
    class Program
    {
        private Font font;

        public Program(string fontPath)
        {
            PrivateFontCollection fonts = new PrivateFontCollection();
            fonts.AddFontFile(fontPath);
            FontFamily family = (FontFamily)fonts.Families.GetValue(0);
            font = new Font(family, 200, FontStyle.Regular);
        }

        public Bitmap Convert(string character)
        {
            Bitmap bitmap = new Bitmap(350, 350);
            Graphics g = Graphics.FromImage(bitmap);
            g.Clear(Color.White);
            g.DrawString(character, font, new SolidBrush(Color.Black), new PointF(10, 10));
            return bitmap;
        }

        static void Main(string[] args)
        {
            string fontName = @"../../simfang.TTF";
            string characterList = @"../../639.txt";
            string GBNumberList = @"../../GB639.txt";
            string outputPath = Path.Combine(@"E:\split", Path.GetFileNameWithoutExtension(fontName));
            if (args.Length > 2)
            {
                fontName = args[1];
                characterList = args[2];
                GBNumberList = args[3];
            }
            Program converter = new Program(fontName);
            string text = File.ReadAllText(characterList);
            Console.WriteLine(text);
            string[] characters = text.Select(c => c.ToString()).ToArray();
            string[] GBNumbers = File.ReadAllLines(GBNumberList);
            if (!Directory.Exists(outputPath))
            {
                Directory.CreateDirectory(outputPath);
            }
            else
            {
                string[] filePaths = Directory.GetFiles(@outputPath);
                foreach (string filePath in filePaths)  File.Delete(filePath);
            }
            for (int i = 0; i < characters.Length;i++ )
            {
                string c = characters[i];
                string no = GBNumbers[i];
                Console.WriteLine(c + ": " + no);
                Bitmap bmp = converter.Convert(c);
                bmp.Save(Path.Combine(@outputPath, no + "_R.bmp"), ImageFormat.Bmp);
                bmp.Dispose();
            }       
        }
    }
}
