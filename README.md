# RBE3001 Matlab Template
This is template code for talking to the Default firmware
# 0.0 Install Matlab

Befor begining install Matlab. 

## 0.1 Matlab Install
* [Matlab Installed using your WPI login](https://www.mathworks.com/academia/tah-portal/worcester-polytechnic-institute-40552010.html)
  * Sign in with your WPI account
  * Download the Linux version R2020a 
  * matlab_R2020a_glnxa64.zip
 ```
 cd Downloads/ #the directory where the matlab_R2020a_glnxa64.zip is
 ls matlab_R2020a_glnxa64.zip # Make sure you see the zip file
 unzip matlab_R2020a_glnxa64.zip -d matlab
 cd matlab
 sudo ./install
 ```
 Sign in with you WPI account again to accuire licences.
 
Under products add 

* Image Acquisition Toolbox
* Image Processing Toolbox

Under Options check the check box

* Make simlinks to MATLAB scripts
* Install MATLAB

## 0.2 Matlab configuration

Open matlab by typing in the terminal

```
matlab
```
Under Environment -> Preferences -> MATLAB -> Keyboard Shortcuts

switch Active  settings from Emacs Default Set to Windows Default Set

## 0.3 Troubleshoting

If you see:

```
License checkout failed.
License Manager Error -9
Your username does not match the username in the license file.
To run on this computer, you must run the Activation client to reactivate your license.

Troubleshoot this issue by visiting:
https://www.mathworks.com/support/lme/R2020a/9

Diagnostic Information:
Feature: MATLAB
License path: /home/mahdi/.matlab/R2020a_licenses:/usr/local/MATLAB/R2020a/licenses/license.dat:/usr/local/MATLAB/
R2020a/licenses/license_mahdi-HP-EliteBook-8470w_40552010_R2020a.lic
Licensing error: -9,57.
```

then run

```
cd /usr/local/MATLAB/R2020a/bin
./activate_matlab.sh
```

# 1. Configure git


#### 1.1 Checking for existing SSH keys
￼
1) Open Terminal.

2) Enter ls -al ~/.ssh to see if existing SSH keys are present:
```
ls -al ~/.ssh
# Lists the files in your .ssh directory, if they exist
```
3) Check the directory listing to see if you already have a public SSH key. By default, the filenames of the public keys are one of the following:
```
id_rsa.pub
id_ecdsa.pub
id_ed25519.pub
```
If the files exist go to 1.3

#### 1.2 Generate SSH key is missing

If there are no files in ~/.ssh then do this step to create them

1) Open Terminal.

2) Paste the text below, substituting in your GitHub email address.
```
ssh-keygen -t rsa -b 4096 -C "your_email@wpi.edu"
```
This creates a new ssh key, using the provided email as a label.
```
> Generating public/private rsa key pair.
```
3) When you're prompted to "Enter a file in which to save the key," press Enter. This accepts the default file location.
```
> Enter a file in which to save the key (/home/you/.ssh/id_rsa): [Press enter]
```
At the prompt, hit enter to make SSh passwordless

#### 1.3 Copy SSH public key

1) Copy the SSH key to your clipboard.

If your SSH key file has a different name than the example code, modify the filename to match your current setup. When copying your key, don't add any newlines or whitespace.
```
sudo apt-get install xclip
# Downloads and installs xclip. If you don't have `apt-get`, you might need to use another installer (like `yum`)

xclip -sel clip < ~/.ssh/id_rsa.pub
# Copies the contents of the id_rsa.pub file to your clipboard
```
2) In the upper-right corner of any page, click your profile photo, then click Settings.


<img src="https://docs.github.com/assets/images/help/settings/userbar-account-settings.png">

3) In the user settings sidebar, click SSH and GPG keys.

<img src="https://docs.github.com/assets/images/help/settings/settings-sidebar-ssh-keys.png">


4) Click New SSH key or Add SSH key.

<img src="https://docs.github.com/assets/images/help/settings/ssh-add-ssh-key.png">


5) In the "Title" field, add a descriptive label for the new key. For example, if you're using a personal Ubuntu, you might call this key "WPI Ubuntu".

6) Paste your key into the "Key" field.

<img src="https://docs.github.com/assets/images/help/settings/ssh-key-paste.png">

7) Click Add SSH key.

<img src="https://docs.github.com/assets/images/help/settings/ssh-add-key.png">

# 1.4 Configure git
```bash
git config --global user.name "John Doe"
git config --global user.email johndoe@wpi.edu
```
# 2. Set up your Git Repository
## 2.1 Clone your private lab repository
Clone your private lab repository. **Note: The command below won't work! Use your own url, found on Github!**
```bash
git clone git@github.com:RBE300X-Lab/RBE3001_MatlabXX.git
```
If your repository is empty, you may get a warning. 

Find your teams project here: https://github.com/RBE300X-Lab

## 2.2 Pulling changes from your secondary remote repository **[Only if required]**
If you need to pull new changes from the default repostory, follow these instructions:
1. Make sure you have set up the correct secondary remote repository. Run the following code to check:
``` bash
git remote -v
```
You should see `origin` (your main server repo) and another pointing to the following url:
```url
https://github.com/Hephaestus-Arm/RBE3001_Matlab.git
```
**If you do not see a second remote, or your second remote points to another url, follow the instructions under [Section 3.2 Part 2](##3.2-Set-up-your-private-lab-repository-**[DO-ONLY-ONCE-PER-TEAM]**)**

2. Run the following command to pull the new code from the secondary remote repository:
``` bash
git pull default-code master
```
Note: If your secondary remote is not named `default-code`, change it to the actual name

# 3. Launch Matlab 

Start in the directory with your checked out code.

```bash
cd src
matlab
```

# 4. Run Lab1.m

Plug your robot into the wall and connect to your computer with USB.
(Protip: make sure putty is closed to avoid latency issues)

In the Matlab GUI navigate to the "lab1.m" file and then click then run button.

Your arm should move through the three setpoints in the viaPoints vector and print some data to the matlab console.

If your arm moves through the setpoints and prints to the console successfully your arm is all set.
