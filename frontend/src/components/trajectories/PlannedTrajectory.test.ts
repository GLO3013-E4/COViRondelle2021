import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import PlannedTrajectory from "@/components/trajectories/PlannedTrajectory.vue";

const vuetify = new Vuetify();
const wrapper = mount(PlannedTrajectory, { vuetify });

describe("When mounting planned trajectory", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
