namespace MovingImitator
{
    partial class Imitation
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.button1 = new System.Windows.Forms.Button();
            this.TelemetricImitator = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.Filtering = new System.Windows.Forms.Button();
            this.Smoothing = new System.Windows.Forms.Button();
            this.RelativeV = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(134, 54);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(109, 49);
            this.button1.TabIndex = 0;
            this.button1.Text = "push the button";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // TelemetricImitator
            // 
            this.TelemetricImitator.Location = new System.Drawing.Point(290, 53);
            this.TelemetricImitator.Name = "TelemetricImitator";
            this.TelemetricImitator.Size = new System.Drawing.Size(125, 50);
            this.TelemetricImitator.TabIndex = 1;
            this.TelemetricImitator.Text = "Telemetric";
            this.TelemetricImitator.UseVisualStyleBackColor = true;
            this.TelemetricImitator.Click += new System.EventHandler(this.TelemetricImitator_Click);
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(471, 194);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 2;
            this.button2.Text = "button2";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // Filtering
            // 
            this.Filtering.Location = new System.Drawing.Point(317, 109);
            this.Filtering.Name = "Filtering";
            this.Filtering.Size = new System.Drawing.Size(75, 23);
            this.Filtering.TabIndex = 3;
            this.Filtering.Text = "Median";
            this.Filtering.UseVisualStyleBackColor = true;
            this.Filtering.Click += new System.EventHandler(this.Filtering_Click);
            // 
            // Smoothing
            // 
            this.Smoothing.Location = new System.Drawing.Point(317, 138);
            this.Smoothing.Name = "Smoothing";
            this.Smoothing.Size = new System.Drawing.Size(75, 23);
            this.Smoothing.TabIndex = 4;
            this.Smoothing.Text = "Smooth";
            this.Smoothing.UseVisualStyleBackColor = true;
            this.Smoothing.Click += new System.EventHandler(this.Smoothing_Click);
            // 
            // RelativeV
            // 
            this.RelativeV.Location = new System.Drawing.Point(317, 167);
            this.RelativeV.Name = "RelativeV";
            this.RelativeV.Size = new System.Drawing.Size(75, 23);
            this.RelativeV.TabIndex = 5;
            this.RelativeV.Text = "Relative V";
            this.RelativeV.UseVisualStyleBackColor = true;
            this.RelativeV.Click += new System.EventHandler(this.RelativeV_Click);
            // 
            // Imitation
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(558, 229);
            this.Controls.Add(this.RelativeV);
            this.Controls.Add(this.Smoothing);
            this.Controls.Add(this.Filtering);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.TelemetricImitator);
            this.Controls.Add(this.button1);
            this.Name = "Imitation";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Form1";
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button TelemetricImitator;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button Filtering;
        private System.Windows.Forms.Button Smoothing;
        private System.Windows.Forms.Button RelativeV;
    }
}

