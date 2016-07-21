
(require 'cl)

(defvar ros-helm--package-path
  (mapconcat 'identity (remove-if-not 'file-exists-p
                                      (split-string
                                       (getenv "ROS_PACKAGE_PATH") ":")) " "))

(defvar ros-helm--launchfile-candidate-list-cache nil)
(defvar ros-helm--service-candidate-list-cache nil)

(defun ros-helm/list-files-of-command (command)
  "Accepts a shell COMMAND that lists files and puts them in a list"
  (with-temp-buffer
    (call-process-shell-command command nil t)
    (goto-char (point-min))
    (let (filelist)
      (while (not (eq 0 (count-lines (point) (point-max))))
        (push (buffer-substring (line-beginning-position) (point)) filelist)
        (end-of-line 2)) ;; move forward a line then go to the end of it
      filelist)))

(defun ros-helm/launchfile-candidate-list ()
  (if ros-helm--launchfile-candidate-list-cache
      ros-helm--launchfile-candidate-list-cache
    (set 'ros-helm--launchfile-candidate-list-cache
          (ros-helm/list-files-of-command
           (format "find %s -type f -name \"*.launch\"" ros-helm--package-path)))))


(defvar helm-source-ros-launchfiles
  (helm-build-sync-source "Launchfiles"
    :candidates (launchfile-candidate-list)
    :action '(("Open file" . (lambda (launchfile-str)
                             (interactive)
                             (find-file launchfile-str))))))

(defun ros-helm/service-candidate-list ()
  (if ros-helm--service-candidate-list-cache
      ros-helm--service-candidate-list-cache
    (set 'ros-helm--service-candidate-list-cache
         (ros-helm/list-files-of-command
          (format "find %s -type f -name \"*.srv\"" ros-helm--package-path)))))

(defvar helm-source-ros-services
  (helm-build-sync-source "Services"
    :candidates (ros-helm/service-candidate-list)
    :action '(("Open file" . (lambda (srvfile-str)
                               (interactive)
                               (find-file srvfile-str))))))
    
(defun ros-helm ()
  (interactive)
  (helm :sources '(helm-source-ros-services helm-source-ros-launchfiles)
        :buffer "*ros-helm*"))

(provide 'ros-helm)
