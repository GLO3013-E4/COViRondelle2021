import CycleInformation from "@/components/cycles/CycleInformation.vue";
import useVuetify from "@/hooks/useVuetify";

const wrapper = useVuetify(CycleInformation);

describe("When mounting cycle information", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
