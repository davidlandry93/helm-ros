
(require 'cl)

(defvar ros-helm--package-path
  (mapconcat 'identity (remove-if-not 'file-exists-p
                                      (split-string
                                       (getenv "ROS_PACKAGE_PATH") ":")) " "))


(defun ros-helm/open-file-action (filename)
  (interactive) (find-file filename))

(defun ros-helm/launch-launchfile (filename)
  (start-process-shell-command "roslaunch"
                               (get-buffer-create "*roslaunch*")
                               (format "roslaunch %s" filename)))

(defun ros-helm/displayed-real-pair-of-path (fullpath)
  (cons (file-name-nondirectory (file-name-sans-extension fullpath)) fullpath))

(defun ros-helm/list-of-command-output (command)
  (with-temp-buffer
    (call-process-shell-command command nil t)
    (split-string (buffer-string) "\n" t)))


;; Launchfiles


(defvar ros-helm--launchfile-candidate-list-cache nil)

(defun ros-helm/launchfile-candidate-list ()
  (if ros-helm--launchfile-candidate-list-cache
      ros-helm--launchfile-candidate-list-cache
    (set 'ros-helm--launchfile-candidate-list-cache
         (mapcar 'ros-helm/displayed-real-pair-of-path
                 (ros-helm/list-of-command-output
                  (format "find -L %s -type f -name \"*.launch\"" ros-helm--package-path))))))


(defvar helm-source-ros-launchfiles
  (helm-build-sync-source "Launchfiles"
    :candidates (ros-helm/launchfile-candidate-list)
    :action '(("Open File" . ros-helm/open-file-action)
              ("Launch" . ros-helm/launch-launchfile))))


;; Services


(defvar ros-helm--service-candidate-list-cache nil)

(defun ros-helm/service-candidate-list ()
  (if ros-helm--service-candidate-list-cache
      ros-helm--service-candidate-list-cache
    (set 'ros-helm--service-candidate-list-cache
         (mapcar 'ros-helm/displayed-real-pair-of-path
                 (ros-helm/list-of-command-output
                  (format "find -L %s -type f -name \"*.srv\"" ros-helm--package-path))))))

(defvar helm-source-ros-services
  (helm-build-sync-source "Services"
    :candidates (ros-helm/service-candidate-list)
    :action '(("Open file" . ros-helm/open-file-action))))


;; Packages


(defvar ros-helm--package-candidate-list-cache nil)

(defun ros-helm/fetch-list-of-packages ()
  (ros-helm/list-of-command-output "rospack list"))

(defun ros-helm/parsed-rospack-entry (entry)
  (let (splitted-string (split-string entry))
    splitted-string))

(defun ros-helm/package-candidate-list ()
  (if ros-helm--package-candidate-list-cache
      ros-helm--package-candidate-list-cache
    (set 'ros-helm--package-candidate-list-cache
         (mapcar (lambda (x)
                   (let ((splitted-string (split-string x)))
                     (cons (car splitted-string) (car (cdr splitted-string)))))
                 (ros-helm/list-of-command-output "rospack list")))))

(defvar helm-source-ros-packages
  (helm-build-sync-source "Packages"
    :candidates (ros-helm/package-candidate-list)
    :action '(("Open folder" . (lambda (candidate) (interactive) (dired candidate))))))

(defun ros-helm ()
  "Launches ros-helm with all available sources."
  (interactive)
  (helm :sources '(helm-source-ros-services
                   helm-source-ros-launchfiles
                   helm-source-ros-packages)
        :buffer "*ros-helm*"))

(provide 'ros-helm)
