// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

/**
 * Tracks whether the topic tree may invoke ROS commands.
 */
export class TopicWatchState {
  private viewVisible = false;
  private watcherEnabled = false;
  private manualRefreshRequested = false;

  public setViewVisible(visible: boolean): void {
    this.viewVisible = visible;
    if (!visible) {
      this.manualRefreshRequested = false;
    }
  }

  public setWatcherEnabled(enabled: boolean): void {
    this.watcherEnabled = enabled;
  }

  public isViewVisible(): boolean {
    return this.viewVisible;
  }

  public isWatcherEnabled(): boolean {
    return this.watcherEnabled;
  }

  public shouldAutoRefresh(windowFocused: boolean): boolean {
    return windowFocused && this.viewVisible && this.watcherEnabled;
  }

  public requestManualRefresh(): boolean {
    if (!this.viewVisible) {
      return false;
    }

    this.manualRefreshRequested = true;
    return true;
  }

  public consumeQueryPermission(): boolean {
    if (!this.viewVisible) {
      return false;
    }

    const permitted = this.watcherEnabled || this.manualRefreshRequested;
    this.manualRefreshRequested = false;
    return permitted;
  }
}
