;;; ros-process-mode.el --- A major mode for ROS process buffers. -*- lexical-binding: t -*-

;; Copyright (C) 2016 David Landry <davidlandry93@gmail.com>

;; This program is free software; you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.

;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.

;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

;;; Code:

(defvar ros-process-mode-hook nil)

(defvar ros-process-mode-map
  (let ((map (make-keymap)))
    (define-key map (kbd "k") 'ros-helm/kill-ros-process)
    (define-key map (kbd "c") 'ros-helm/interrupt-ros-process)
    (define-key map (kbd "q") (lambda () (interactive) (delete-window)))
    map)
  "Keymap for the ros-node major mode")

(defun ros-helm/interrupt-ros-process ()
  "Interrupts the ros process associated with the current buffer."
  (interactive)
  (let ((ros-node-process (get-buffer-process (current-buffer))))
    (interrupt-process ros-node-process)))

(defun ros-helm/kill-ros-process ()
  "Kills the ros process associated with the current buffer."
  (interactive)
  (let ((ros-node-process (get-buffer-process (current-buffer))))
    (kill-process ros-node-process)))

(defun ros-helm//ros-process-filter (process string)
  "Applies `xterm-color-filter' to the text before outputting it
to the process buffer."
  (when (buffer-live-p (process-buffer process))
    (with-current-buffer (process-buffer process)
      (let ((moving (= (point) (process-mark process))))
        (save-excursion
          (goto-char (process-mark process))
          (insert (xterm-color-filter string))
          (set-marker (process-mark process) (point)))
        (if moving (goto-char (process-mark process)))))))

(define-derived-mode ros-process-mode fundamental-mode "ROS Node Mode"
  "Major mode for handling a rosrun console."
  (set-process-filter (get-buffer-process (current-buffer)) 'ros-helm//ros-process-filter))

(provide 'ros-process-mode)

;;; ros-process-mode.el ends here
