
(require 'cl)

(defvar ros-helm--package-path
  (mapconcat 'identity (remove-if-not 'file-exists-p
                                      (split-string
                                       (getenv "ROS_PACKAGE_PATH") ":")) " "))

(defvar ros-helm--launchfile-candidate-list-cache nil)
(defvar ros-helm--service-candidate-list-cache nil)

(defun ros-helm/open-file-action (filename)
  (interactive) (find-file filename))

(defun ros-helm/launch-launchfile (filename)
  (start-process-shell-command "roslaunch"
                               (get-buffer-create "*roslaunch*")
                               (format "roslaunch %s" filename)))

(defun ros-helm/displayed-real-pair-of-path (fullpath)
  `(,(file-name-nondirectory (file-name-sans-extension fullpath)) . ,fullpath))

(defun ros-helm/list-files-of-command (command)
  "Accepts a shell COMMAND that lists files and outputs
a list of (displayed . real) candidate name."
  (with-temp-buffer
    (call-process-shell-command command nil t)
    (goto-char (point-min))
    (let (filelist)
      (while (not (eq 0 (count-lines (point) (point-max))))
        (push
         (let ((fullpath (buffer-substring (line-beginning-position) (point))))
           (ros-helm/displayed-real-pair-of-path fullpath))
         filelist)
        (end-of-line 2)) ;; move forward a line then go to the end of it
      filelist)))

(defun ros-helm/launchfile-candidate-list ()
  (if ros-helm--launchfile-candidate-list-cache
      ros-helm--launchfile-candidate-list-cache
    (set 'ros-helm--launchfile-candidate-list-cache
          (ros-helm/list-files-of-command
           (format "find -L %s -type f -name \"*.launch\"" ros-helm--package-path)))))


(defvar helm-source-ros-launchfiles
  (helm-build-sync-source "Launchfiles"
    :candidates (ros-helm/launchfile-candidate-list)
    :action '(("Open File" . ros-helm/open-file-action)
              ("Launch" . ros-helm/launch-launchfile))))

(defun ros-helm/service-candidate-list ()
  (if ros-helm--service-candidate-list-cache
      ros-helm--service-candidate-list-cache
    (set 'ros-helm--service-candidate-list-cache
         (ros-helm/list-files-of-command
          (format "find -L %s -type f -name \"*.srv\"" ros-helm--package-path)))))

(defvar helm-source-ros-services
  (helm-build-sync-source "Services"
    :candidates (ros-helm/service-candidate-list)
    :action '(("Open file" . ros-helm/open-file-action))))

(defun ros-helm ()
  (interactive)
  (helm :sources '(helm-source-ros-services helm-source-ros-launchfiles)
        :buffer "*ros-helm*"))

(provide 'ros-helm)
