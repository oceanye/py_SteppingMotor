const {defineConfig, devices} = require('@playwright/test');
const path = require('node:path');

// No hardware server reuse. Override PYTHON with an installed Python >=3.10.
const python = process.env.PLAYWRIGHT_PYTHON || 'python';
const port = 18765;
module.exports = defineConfig({
  testDir: './tests/e2e',
  timeout: 20000,
  expect: {timeout: 6000},
  fullyParallel: false,
  workers: 2,
  retries: 0,
  forbidOnly: !!process.env.CI,
  reporter: [['list'], ['html', {open: 'never'}], ['json', {outputFile: 'test-results/results.json'}]],
  use: {
    baseURL: `http://127.0.0.1:${port}`,
    trace: 'retain-on-failure',
    screenshot: 'only-on-failure',
    actionTimeout: 6000,
  },
  projects: [
    {name: 'chromium', use: {...devices['Desktop Chrome']}},
    {name: 'firefox', use: {...devices['Desktop Firefox']}},
    {name: 'webkit', use: {...devices['Desktop Safari']}},
    {name: 'mobile-chromium', use: {...devices['Pixel 7']}},
    {name: 'edge', use: {...devices['Desktop Edge'], channel: 'msedge'}},
  ],
  webServer: {
    command: `"${python}" -B tests/e2e/server.py --port ${port}`,
    cwd: path.resolve(__dirname),
    url: `http://127.0.0.1:${port}/api/status`,
    reuseExistingServer: false,
    timeout: 30000,
    env: {PYTHONIOENCODING: 'utf-8'},
  },
});
