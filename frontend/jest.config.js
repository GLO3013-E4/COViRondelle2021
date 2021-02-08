module.exports = {
  preset: "@vue/cli-plugin-unit-jest/presets/typescript-and-babel",
  transform: {
    "^.+\\.vue$": "vue-jest",
  },
  testMatch: ["**/*.test*.ts?(x)"],
  setupFiles: ['./src/setupTests.ts']
};
